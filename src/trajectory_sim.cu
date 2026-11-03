#include "trajectory_generator.cuh"
#include "mppi_node.h"
#include <curand_kernel.h>

// Constants for GPU
#define THREADS_PER_BLOCK 256

// Noise generation kernel
__global__ void generate_noise_kernel(float* d_noise, int num_trajs, int horizon) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx >= num_trajs * horizon * 2) return;
    
    // Simple uniform random for now (replace with curand for better randomness)
    curandState state;
    curand_init(clock64(), idx, 0, &state);
    
    int traj_id = idx / (horizon * 2);
    int param_id = idx % (horizon * 2);
    
    if(param_id % 2 == 0) {
        // Steering noise
        d_noise[idx] = curand_normal(&state) * RANDOM_STEERING;
    } else {
        // Acceleration noise
        d_noise[idx] = curand_normal(&state) * RANDOM_ACCEL;
    }
}

// Dynamics simulation + cost calculation kernel
__global__ void simulate_and_cost_kernel(
    float* d_noise,
    float* d_states,
    float* d_costs,
    float init_x, float init_y, float init_theta, float init_v,
    int num_trajs,
    int horizon
) {
    int traj_id = blockIdx.x * blockDim.x + threadIdx.x;
    if(traj_id >= num_trajs) return;
    
    // Initialize state
    float x = init_x, y = init_y, theta = init_theta, v = init_v;
    float total_cost = 0.0f;
    float dt = 0.01f;  // timestep
    float L = 0.33f;   // wheelbase (F1TENTH typical)
    
    for(int t = 0; t < horizon; t++) {
        // Get control inputs (noise)
        int noise_idx = traj_id * horizon * 2 + t * 2;
        float steering = d_noise[noise_idx];
        float accel = d_noise[noise_idx + 1];
        
        // Kinematic bicycle model
        x += v * cosf(theta) * dt;
        y += v * sinf(theta) * dt;
        theta += (v / L) * tanf(steering) * dt;
        v += accel * dt;
        
        // Clamp velocity
        v = fmaxf(-5.0f, fminf(v, 8.0f));
        
        // Store state
        int state_idx = traj_id * horizon * 4 + t * 4;
        d_states[state_idx + 0] = x;
        d_states[state_idx + 1] = y;
        d_states[state_idx + 2] = theta;
        d_states[state_idx + 3] = v;
        
        // Compute cost (simplified for now)
        float step_cost = 0.0f;
        
        // Rough collision detection (bounds check)
        if(x < -5.0f || x > 5.0f || y < -5.0f || y > 5.0f) {
            step_cost += W_COLLISION * 100.0f;
        }
        
        // Control smoothness
        step_cost += W_CONTROL * (fabsf(steering) + fabsf(accel));
        
        // Speed reward
        step_cost += W_SPEED * (8.0f - v);
        
        total_cost += step_cost;
    }
    
    d_costs[traj_id] = total_cost;
}

// Allocate GPU memory
device_trajectories allocate_device_memory() {
    device_trajectories dev;
    size_t noise_size = NUM_TRAJECTORIES * PREDICTION_HORIZON * 2 * sizeof(float);
    size_t states_size = NUM_TRAJECTORIES * PREDICTION_HORIZON * 4 * sizeof(float);
    size_t costs_size = NUM_TRAJECTORIES * sizeof(float);
    
    cudaMalloc(&dev.d_noise, noise_size);
    cudaMalloc(&dev.d_states, states_size);
    cudaMalloc(&dev.d_costs, costs_size);
    
    return dev;
}

// Free GPU memory
void free_device_memory(device_trajectories& dev) {
    cudaFree(dev.d_noise);
    cudaFree(dev.d_states);
    cudaFree(dev.d_costs);
}

// Run the full GPU pipeline
void run_gpu_pipeline(device_trajectories& dev, carState state) {
    int blocks = (NUM_TRAJECTORIES * PREDICTION_HORIZON * 2 + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    generate_noise_kernel<<<blocks, THREADS_PER_BLOCK>>>(dev.d_noise, NUM_TRAJECTORIES, PREDICTION_HORIZON);
    
    blocks = (NUM_TRAJECTORIES + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    simulate_and_cost_kernel<<<blocks, THREADS_PER_BLOCK>>>(
        dev.d_noise, dev.d_states, dev.d_costs,
        state.x, state.y, state.theta, state.v,
        NUM_TRAJECTORIES, PREDICTION_HORIZON
    );
    
    cudaDeviceSynchronize();  // Wait for GPU to finish
}