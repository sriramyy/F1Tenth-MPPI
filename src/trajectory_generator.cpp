#include <iostream>
#include "trajectory_generator.h"
#include "mppi_node.h"
#include <list>
#include <queue>
#include <unordered_map>
#include <vector>
#include <utility>
#include "trajectory_sim.cuh"

// this file should 
//  generate random trajectory noise -> evaluate the dynamics at each timestep -> calculate the cost at each timestep

using std::list, std::priority_queue, std::unordered_map, std::vector, std::pair;


// returns the costs of all of the trajectories in a max heap (using CUDA to run in parallel)
all_trajectories generate_trajectories_and_costs(carState state) {
    // alloc the gpu memory
    device_trajectories dev = allocate_device_memory();

    // run the gpu pipeline
    run_gpu_pipeline(dev, state);

    // copy results back here (CPU)
    float* h_costs = new float[NUM_TRAJECTORIES];
    float* h_noise = new float[NUM_TRAJECTORIES * PREDICTION_HORIZON * 2];

    cudaMemcpy(h_costs, dev.d_costs, NUM_TRAJECTORIES * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_noise, dev.d_noise, NUM_TRAJECTORIES * PREDICTION_HORIZON * 2 * sizeof(float), cudaMemcpyDeviceToHost);

    // put into our structs
    all_trajectories result;
    result.trajs.resize(NUM_TRAJECTORIES);
    
    for(int i = 0; i < NUM_TRAJECTORIES; i++) {
        result.trajs[i].cost = h_costs[i];
        result.trajs[i].input.resize(PREDICTION_HORIZON);
        
        for(int t = 0; t < PREDICTION_HORIZON; t++) {
            int noise_idx = i * PREDICTION_HORIZON * 2 + t * 2;
            result.trajs[i].input[t].steering = h_noise[noise_idx];
            result.trajs[i].input[t].acceleration = h_noise[noise_idx + 1];
        }
        
        result.costs.push({h_costs[i], i});
    }
    
    // delete the original
    delete[] h_costs;
    delete[] h_noise;
    free_device_memory(dev);
    
    return result;
}

// gets the top n trajectories and average both the accel and steering of each to return the average trajectory
trajectory get_average_trajectory(all_trajectories all) {
    trajectory avg;
    avg.input.resize(PREDICTION_HORIZON);
    
    for(int t = 0; t < PREDICTION_HORIZON; t++) {
        float avg_steering = 0, avg_accel = 0;
        for(int i = 0; i < TOP_TRAJECTORIES_NUM && !all.costs.empty(); i++) {
            auto [cost, idx] = all.costs.top();
            all.costs.pop();
            avg_steering += all.trajs[idx].input[t].steering;
            avg_accel += all.trajs[idx].input[t].acceleration;
        }
        avg.input[t].steering = avg_steering / TOP_TRAJECTORIES_NUM;
        avg.input[t].acceleration = avg_accel / TOP_TRAJECTORIES_NUM;
    }
    
    return avg;
}



