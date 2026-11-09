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
    
    // Initialize state
    current_state_ = {0.0f, 0.0f, 0.0f, 0.0f};
    
    RCLCPP_INFO(node_->get_logger(), "MPPI Controller initialized");
}

void rosFunctions::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract position
    std::lock_guard<std::mutex> lock(state_mutex_);

    current_state_.x = msg->pose.pose.position.x;
    current_state_.y = msg->pose.pose.position.y;
    current_state_.v = msg->twist.twist.linear.x;
    
    // Extract orientation (quaternion to yaw)
    double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                            msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    current_state_.theta = std::atan2(siny_cosp, cosy_cosp);
}

bool rosFunctions::execute_trajectory(trajectory traj) {
    if(traj.input.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Empty trajectory received");
        return false;
    }
    
    // Execute first control input from trajectory
    // (you'd typically execute the whole trajectory at control_frequency Hz)
    auto cmd = geometry_msgs::msg::Twist();
    
    cmd.linear.x = traj.input[0].acceleration;   // Forward velocity
    cmd.angular.z = traj.input[0].steering;      // Steering angle
    
    cmd_vel_pub_->publish(cmd);
    
    RCLCPP_DEBUG(node_->get_logger(), 
        "Executed control: accel=%.3f, steering=%.3f",
        traj.input[0].acceleration, traj.input[0].steering
    );
    
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