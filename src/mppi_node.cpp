#include "mppi_node.h"
#include "trajectory_generator.h"
#include "track_handler.h"
#include "ros_functions.h"

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    rosFunctions ros;

    track_handler = new TrackHandler(MAP_CENTERLINE_FILE, MAP_YAML_FILE);

    auto last_execution = std::chrono::steady_clock::now();
    auto control_period = std::chrono::milliseconds(50); // 50ms per step
    
    // store car state
    carState state; 
    bool master_running = true;
    int trajectory_index = 0;

    while (master_running) {
        state = ros.get_current_car_state();

        auto now = std::chrono::steady_clock::now();
        if(now - last_execution >= control_period) {
            
            if(trajectory_index >= PREDICTION_HORIZON) {
                // Time the trajectory generation
                auto gen_start = std::chrono::steady_clock::now();
                
                auto all_trajs = generate_trajectories_and_costs(state);
                current_trajectory = get_average_trajectory(all_trajs);
                
                auto gen_end = std::chrono::steady_clock::now();
                auto gen_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    gen_end - gen_start
                ).count();
                
                // Print with formatting
                ros.print(std::format(
                    "gen complete, took {}ms for {} trajectories", 
                    gen_ms, NUM_TRAJECTORIES
                ));
                
                trajectory_index = 0;
            }
            
            trajectory single_step;
            single_step.input.resize(1);
            single_step.input[0] = current_trajectory.input[trajectory_index];
            ros.execute_trajectory(single_step);
            
            ros.print(std::format(
                "exec step {}/{}: steering={:.3f}, accel={:.3f}",
                trajectory_index, PREDICTION_HORIZON,
                single_step.input[0].steering,
                single_step.input[0].acceleration
            ));
            
            trajectory_index++;
            last_execution = now;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    delete track_handler;
    rclcpp::shutdown();
    return 0;
}