#pragma once


//
struct trajectory {
    struct control_input {
        float steering;
        float acceleration;
    };

    // cost of the trajectory is already present within the hashmap containing <COST, trajectory>
    vector<control_input> input; // control input at EACH timestep
};

// contains all trajectories in a hashamp indexed by cost and a max heap of the costs
struct all_trajectories {
    priority_queue<int> costs; // max heap of the costs
    unordered_map<int, trajectory> map; 
};

// main funciton that runs the CUDA processes and returns all of the generated trajectories along with their costs
all_trajectories generate_trajectories_and_costs();

// helper functions
trajectory get_average_trajectory(all_trajectories all);
