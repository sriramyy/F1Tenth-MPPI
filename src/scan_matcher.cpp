#include "scan_matcher.h"
#include <cmath>

ScanMatcher::ScanMatcher() : first_scan_(true) {
    last_odometry_ = {0.0f, 0.0f, 0.0f, 0.0f};
}

carState ScanMatcher::update(const carState& odometry_estimate, const LaserScan& current_scan) {
    if(first_scan_) {
        previous_scan_ = current_scan;
        last_odometry_ = odometry_estimate;
        first_scan_ = false;
        return odometry_estimate;
    }
    
    // Simple scan matching: find best alignment of current scan to previous scan
    float best_error = 1e6f;
    float best_dx = 0.0f, best_dy = 0.0f, best_dtheta = 0.0f;
    
    // Search over possible transformations
    for(float dx = -0.1f; dx <= 0.1f; dx += 0.02f) {
        for(float dy = -0.1f; dy <= 0.1f; dy += 0.02f) {
            for(float dtheta = -0.1f; dtheta <= 0.1f; dtheta += 0.02f) {
                float error = 0.0f;
                
                // Calculate error between transformed current scan and previous scan
                for(size_t i = 0; i < current_scan.ranges.size(); i++) {
                    float angle = current_scan.angle_min + i * current_scan.angle_increment;
                    
                    // Transform current scan point
                    float x = current_scan.ranges[i] * cosf(angle);
                    float y = current_scan.ranges[i] * sinf(angle);
                    
                    float x_transformed = (x + dx) * cosf(dtheta) - (y + dy) * sinf(dtheta);
                    float y_transformed = (x + dx) * sinf(dtheta) + (y + dy) * cosf(dtheta);
                    
                    // Find nearest point in previous scan
                    float min_dist = 1e6f;
                    for(size_t j = 0; j < previous_scan_.ranges.size(); j++) {
                        float prev_angle = previous_scan_.angle_min + j * previous_scan_.angle_increment;
                        float px = previous_scan_.ranges[j] * cosf(prev_angle);
                        float py = previous_scan_.ranges[j] * sinf(prev_angle);
                        
                        float dx_p = x_transformed - px;
                        float dy_p = y_transformed - py;
                        float dist = sqrtf(dx_p * dx_p + dy_p * dy_p);
                        min_dist = fminf(min_dist, dist);
                    }
                    error += min_dist;
                }
                
                if(error < best_error) {
                    best_error = error;
                    best_dx = dx;
                    best_dy = dy;
                    best_dtheta = dtheta;
                }
            }
        }
    }
    
    // Correct odometry with scan matching result
    carState corrected_state = odometry_estimate;
    corrected_state.x += best_dx;
    corrected_state.y += best_dy;
    corrected_state.theta += best_dtheta;
    
    previous_scan_ = current_scan;
    last_odometry_ = corrected_state;
    
    return corrected_state;
}