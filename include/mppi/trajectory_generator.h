#pragma once

#include "ros_functions.h"

//
struct trajectory {
    struct control_input {
        float steering;
        float acceleration;
    };

    vector<control_input> input; // control input at EACH timestep
    float cost;
};

// contains all trajectories in a hashamp indexed by cost and a max heap of the costs
struct all_trajectories {
    priority_queue<pair<float,int>> costs; // max heap: (cost, trajectory index)
    vector<trajectory> trajs;    // stored based on index
};

// main funciton that runs the CUDA processes and returns all of the generated trajectories along with their costs
all_trajectories generate_trajectories_and_costs(carState state);

// helper functions
trajectory get_average_trajectory(all_trajectories all);
