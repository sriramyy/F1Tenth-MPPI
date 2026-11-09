#pragma once
#include <string>
#include <vector>

struct CenterlinePoint {
    float x;
    float y;
};

class TrackHandler {
private:
    std::vector<CenterlinePoint> centerline;
    float track_bounds_x_min, track_bounds_x_max;
    float track_bounds_y_min, track_bounds_y_max;
    
public:
    TrackHandler(const std::string& centerline_file, const std::string& yaml_file);
    float get_centerline_distance(float x, float y) const;
    bool is_in_bounds(float x, float y) const;
    size_t get_centerline_size() const { return centerline.size(); }
    const std::vector<CenterlinePoint>& get_centerline() const { return centerline; }
};