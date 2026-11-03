#pragma once

// ------------------------ CONSTANTS ------------------------ //
const int PREDICTION_HORIZON {50};    // time steps into the future each prediction horizon is
const int NUM_TRAJECTORIES   {1000};  // number of trajectories to generate for each prediction horizon

// sigma random functions when generating trajectories 
const float RANDOM_ACCEL    {0.5};      // sigma for acceleration random generation
const float RANDOM_STEERING {0.3};      // sigma for steering angle random generation

// cost function weights
const float W_COLLISION  {100.0};
const float W_TRACK      {10.0}; 
const float W_CONTROL    {0.5}; 
const float W_SPEED      {5.0};

//
const int TOP_TRAJECTORIES_NUM {50}; // number of trajectories to choose when averaging the final trajectory

//

int main();