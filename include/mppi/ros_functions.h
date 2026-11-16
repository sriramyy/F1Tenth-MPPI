#pragma once
#include "trajectory_generator.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "scan_matcher.h"
#include <memory>
#include <mutex>

struct carState {
    float x;
    float y;
    float theta;
    float v;
};

class rosFunctions {
private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    carState current_state_;
    carState odometry_estimate_;
    ScanMatcher* scan_matcher_;
    mutable std::mutex state_mutex_;
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
public:
    rosFunctions();
    
    bool execute_trajectory(trajectory traj);
    carState get_current_car_state();
    void rosprint(const std::string& message);
};