#include "track_handler.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

TrackHandler::TrackHandler(const std::string& centerline_file, const std::string& yaml_file) {
    // Load centerline from CSV
    std::ifstream csv_file(centerline_file);
    if(!csv_file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("TrackHandler"), 
                     "Could not open centerline file: %s", centerline_file.c_str());
        return;
    }
    
    std::string line;
    while(std::getline(csv_file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str;
        
        if(std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
            try {
                float x = std::stof(x_str);
                float y = std::stof(y_str);
                centerline.push_back({x, y});
            } catch(...) {
                continue;  // Skip malformed lines
            }
        }
    }
    csv_file.close();
    
    // Load track bounds from YAML
    try {
        YAML::Node config = YAML::LoadFile(yaml_file);
        track_bounds_x_min = config["track_bounds"]["x_min"].as<float>(-5.0f);
        track_bounds_x_max = config["track_bounds"]["x_max"].as<float>(5.0f);
        track_bounds_y_min = config["track_bounds"]["y_min"].as<float>(-5.0f);
        track_bounds_y_max = config["track_bounds"]["y_max"].as<float>(5.0f);
        
        RCLCPP_INFO(rclcpp::get_logger("TrackHandler"),
                    "Loaded track with %zu centerline points", centerline.size());
    } catch(const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("TrackHandler"),
                    "Failed to load YAML config, using defaults: %s", e.what());
        track_bounds_x_min = -5.0f;
        track_bounds_x_max = 5.0f;
        track_bounds_y_min = -5.0f;
        track_bounds_y_max = 5.0f;
    }
}

float TrackHandler::get_centerline_distance(float x, float y) const {
    if(centerline.empty()) return 0.0f;
    
    float min_dist = 1e6;
    for(const auto& point : centerline) {
        float dx = x - point.x;
        float dy = y - point.y;
        float dist = std::sqrt(dx * dx + dy * dy);
        if(dist < min_dist) min_dist = dist;
    }
    return min_dist;
}

bool TrackHandler::is_in_bounds(float x, float y) const {
    return x >= track_bounds_x_min && x <= track_bounds_x_max &&
           y >= track_bounds_y_min && y <= track_bounds_y_max;
}