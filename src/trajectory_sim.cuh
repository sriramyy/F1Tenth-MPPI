#pragma once
#include "trajectory_generator.h"
#include <curand_kernel.h>

// Constants for GPU
#define THREADS_PER_BLOCK 256

// Device memory structures
struct device_trajectories {
    float* d_noise;           // shape: [NUM_TRAJECTORIES][PREDICTION_HORIZON][2]
    float* d_states;          // shape: [NUM_TRAJECTORIES][PREDICTION_HORIZON][4]
    float* d_costs;           // shape: [NUM_TRAJECTORIES]
    bool* d_collision_flags;  // track which trajectories collided
    float* d_centerline_x;    // centerline x coordinates
    float* d_centerline_y;    // centerline y coordinates
};

// CUDA kernel declarations
__global__ void generate_noise_kernel(float* d_noise, int num_trajs, int horizon);

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
);

// Wrapper functions
device_trajectories allocate_device_memory(int centerline_size);
void free_device_memory(device_trajectories& dev);
void run_gpu_pipeline(device_trajectories& dev, carState state, int centerline_size);