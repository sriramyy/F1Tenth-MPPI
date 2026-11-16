#include "ros_functions.h"
#include <cmath>

private:
    mutable std::mutex state_mutex_;


rosFunctions::rosFunctions() {
    // Initialize ROS 2 node
    node_ = std::make_shared<rclcpp::Node>("mppi_controller");
    
    // Create publisher for velocity commands
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10
    );
    
    // Create subscriber for odometry (car state)
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&rosFunctions::odom_callback, this, std::placeholders::_1)
    );
    
    laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&rosFunctions::laser_callback, this, std::placeholders::_1)
    );

    scan_matcher_ = new ScanMatcher();
    current_state_ = {0.0f, 0.0f, 0.0f, 0.0f};

    // Initialize state
    current_state_ = {0.0f, 0.0f, 0.0f, 0.0f};
    
    RCLCPP_INFO(node_->get_logger(), "MPPI Controller initialized");
}

void rosFunctions::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    odometry_estimate_.x = msg->pose.pose.position.x;
    odometry_estimate_.y = msg->pose.pose.position.y;
    odometry_estimate_.v = msg->twist.twist.linear.x;
    
    double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                            msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    odometry_estimate_.theta = std::atan2(siny_cosp, cosy_cosp);
}

void rosFunctions::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    LaserScan scan;
    scan.ranges.assign(msg->ranges.begin(), msg->ranges.end());
    scan.angle_min = msg->angle_min;
    scan.angle_max = msg->angle_max;
    scan.angle_increment = msg->angle_increment;
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    // Correct odometry with scan matching
    current_state_ = scan_matcher_->update(odometry_estimate_, scan);
}

bool rosFunctions::execute_trajectory(trajectory traj) {
    if(traj.input.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Empty trajectory received");
        return false;
    }
    
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = traj.input[0].acceleration;
    cmd.angular.z = traj.input[0].steering;
    cmd_vel_pub_->publish(cmd);
    
    return true;
}

carState rosFunctions::get_current_car_state() {
    rclcpp::spin_some(node_);
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_;
}

void rosFunctions::print(const std::string& message) {
    RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
}

void rosFunctions::print_info(const std::string& message) {
    RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
}

void rosFunctions::print_warn(const std::string& message) {
    RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
}

void rosFunctions::print_error(const std::string& message) {
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
}