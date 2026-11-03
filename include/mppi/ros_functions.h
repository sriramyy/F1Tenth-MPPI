#pragma once

#include "trajectory_generator.h"

//

struct carState {
    float x;
    float y;
    float theta;
    float v;
};

class rosFunctions {
public:
    rosFunctions() {};

    bool execute_trajectory(trajectory traj);

    carState get_current_car_state();


};