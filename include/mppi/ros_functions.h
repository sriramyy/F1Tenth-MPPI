#pragma once

#include "trajectory_generator.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <memory>

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
    carState current_state_;
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

public:
    rosFunctions();
    
    bool execute_trajectory(trajectory traj);
    carState get_current_car_state();

    // logging functions
    void print(const std::string& message);
    void print_info(const std::string& message);
    void print_warn(const std::string& message);
    void print_error(const std::string& message);
};