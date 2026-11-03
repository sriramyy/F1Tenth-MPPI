#pragma once
#include "trajectory_generator.h"

// Device memory structures
struct device_trajectories {
    float* d_noise;           // shape: [NUM_TRAJECTORIES][PREDICTION_HORIZON][2]
    float* d_states;          // shape: [NUM_TRAJECTORIES][PREDICTION_HORIZON][4]
    float* d_costs;           // shape: [NUM_TRAJECTORIES]
};

// CUDA kernel declarations
__global__ void generate_noise_kernel(float* d_noise, int num_trajs, int horizon);

__global__ void simulate_and_cost_kernel(
    float* d_noise, 
    float* d_states, 
    float* d_costs,
    float init_x, float init_y, float init_theta, float init_v,
    int num_trajs, 
    int horizon
);

// Wrapper functions (call these from .cpp)
device_trajectories allocate_device_memory();
void free_device_memory(device_trajectories& dev);
void run_gpu_pipeline(device_trajectories& dev, carState state);