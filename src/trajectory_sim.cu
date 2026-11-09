#include "trajectory_sim.cuh"
#include "mppi_node.h"
#include <curand_kernel.h>
#include <cmath>

// Noise generation kernel
__global__ void generate_noise_kernel(float* d_noise, int num_trajs, int horizon) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx >= num_trajs * horizon * 2) return;
    
    curandState state;
    curand_init(clock64() + idx, idx, 0, &state);
    
    int param_id = idx % 2;
    
    if(param_id == 0) {
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
    bool* d_collision_flags,
    float init_x, float init_y, float init_theta, float init_v,
    float* d_centerline_x,
    float* d_centerline_y,
    int centerline_size,
    int num_trajs,
    int horizon
) {
    int traj_id = blockIdx.x * blockDim.x + threadIdx.x;
    if(traj_id >= num_trajs) return;
    
    // Initialize state
    float x = init_x, y = init_y, theta = init_theta, v = init_v;
    float total_cost = 0.0f;
    float dt = 0.01f;
    float L = 0.33f;
    bool collision_detected = false;
    
    for(int t = 0; t < horizon; t++) {
        // If collision already detected, skip remaining timesteps
        if(collision_detected) {
            break;
        }
        
        // Get control inputs (noise) with zero reference control
        int noise_idx = traj_id * horizon * 2 + t * 2;
        float ref_steering = 0.0f;  // Zero control reference
        float ref_accel = 0.0f;     // Zero control reference
        
        float steering = ref_steering + d_noise[noise_idx];
        float accel = ref_accel + d_noise[noise_idx + 1];
        
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
        
        // Compute cost
        float step_cost = 0.0f;
        
        // Collision detection with early termination
        if(x < -5.0f || x > 5.0f || y < -5.0f || y > 5.0f) {
            total_cost += W_COLLISION * 10000.0f;  // Huge penalty
            collision_detected = true;
            break;
        }
        
        // Compute centerline distance at THIS timestep
        float min_centerline_dist = 1e6f;
        if(centerline_size > 0) {
            for(int i = 0; i < centerline_size; i++) {
                float dx = x - d_centerline_x[i];
                float dy = y - d_centerline_y[i];
                float dist = sqrtf(dx * dx + dy * dy);
                min_centerline_dist = fminf(min_centerline_dist, dist);
            }
        }
        
        // Control smoothness
        step_cost += W_CONTROL * (fabsf(steering) + fabsf(accel));
        
        // Track progress (centerline distance)
        step_cost += W_TRACK * min_centerline_dist;
        
        // Speed reward
        step_cost += W_SPEED * (8.0f - v);
        
        total_cost += step_cost;
    }
    
    d_costs[traj_id] = total_cost;
    d_collision_flags[traj_id] = collision_detected;
}

// Allocate GPU memory
device_trajectories allocate_device_memory(int centerline_size) {
    device_trajectories dev;
    size_t noise_size = NUM_TRAJECTORIES * PREDICTION_HORIZON * 2 * sizeof(float);
    size_t states_size = NUM_TRAJECTORIES * PREDICTION_HORIZON * 4 * sizeof(float);
    size_t costs_size = NUM_TRAJECTORIES * sizeof(float);
    size_t flags_size = NUM_TRAJECTORIES * sizeof(bool);
    size_t centerline_size_bytes = centerline_size * sizeof(float);
    
    cudaMalloc(&dev.d_noise, noise_size);
    cudaMalloc(&dev.d_states, states_size);
    cudaMalloc(&dev.d_costs, costs_size);
    cudaMalloc(&dev.d_collision_flags, flags_size);
    cudaMalloc(&dev.d_centerline_x, centerline_size_bytes);
    cudaMalloc(&dev.d_centerline_y, centerline_size_bytes);
    
    return dev;
}

// Free GPU memory
void free_device_memory(device_trajectories& dev) {
    cudaFree(dev.d_noise);
    cudaFree(dev.d_states);
    cudaFree(dev.d_costs);
    cudaFree(dev.d_collision_flags);
    cudaFree(dev.d_centerline_x);
    cudaFree(dev.d_centerline_y);
}

// Run the full GPU pipeline
void run_gpu_pipeline(device_trajectories& dev, carState state, int centerline_size) {
    // Generate noise
    int blocks = (NUM_TRAJECTORIES * PREDICTION_HORIZON * 2 + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    generate_noise_kernel<<<blocks, THREADS_PER_BLOCK>>>(dev.d_noise, NUM_TRAJECTORIES, PREDICTION_HORIZON);
    
    // Simulate and calculate costs
    blocks = (NUM_TRAJECTORIES + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    simulate_and_cost_kernel<<<blocks, THREADS_PER_BLOCK>>>(
        dev.d_noise,
        dev.d_states,
        dev.d_costs,
        dev.d_collision_flags,
        state.x, state.y, state.theta, state.v,
        dev.d_centerline_x,
        dev.d_centerline_y,
        centerline_size,
        NUM_TRAJECTORIES,
        PREDICTION_HORIZON
    );
    
    cudaDeviceSynchronize();
}