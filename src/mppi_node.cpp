#include "mppi_node.h"
#include "trajectory_generator.h"
#include "ros_functions.h"

int main() {

    bool master_running{1};
    int horizon_counter{PREDICTION_HORIZON};

    rosFunctions ros;
    
    // store car state
    carState state; 

    while (master_running) {

        // update car state
        state = ros.get_current_car_state();

        if (horizon_counter == 0) {

            // get all of the trajectories for this prediction horizon
            auto all_trajs = generate_trajectories_and_costs(state);
            
            // get the average trajectory we need to execute
            auto traj = get_average_trajectory(all_trajs);

            // execute the average trajectory 
            ros.execute_trajectory(traj);


            // reset the counter until the next horizon
            horizon_counter = PREDICTION_HORIZON;

        } else {
            horizon_counter--;
        }
    }
}


