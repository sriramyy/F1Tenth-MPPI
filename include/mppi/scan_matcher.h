#pragma once
#include "ros_functions.h"
#include <vector>
#include <Eigen/Dense>

struct LaserScan {
    std::vector<float> ranges;
    float angle_min;
    float angle_max;
    float angle_increment;
};

class ScanMatcher {
private:
    LaserScan previous_scan_;
    carState last_odometry_;
    bool first_scan_;
    
public:
    ScanMatcher();
    carState update(const carState& odometry_estimate, const LaserScan& current_scan);
    void process_laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};