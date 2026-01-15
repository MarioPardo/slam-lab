#include "lidar_processor.h"
#include <cmath>
#include <limits>

namespace slam {

LidarProcessor::LidarProcessor() {}

//Transform LIDAR a whole lidar scan from robot frame to world frame
std::vector<Point2D> LidarProcessor::transformToWorld(const LidarScan& scan, const Pose2D& robot_pose) {
    std::vector<Point2D> world_points;
    world_points.reserve(scan.count);
    
    if (scan.ranges.empty()) {
        return world_points;
    }
    
    double angle_increment = (scan.angle_max - scan.angle_min) / (scan.count - 1);
    
    for (int i = 0; i < scan.count; i++) 
    {
        double range = scan.ranges[i];
        
        if (std::isinf(range) || std::isnan(range)) 
            continue;
        
        if (range < scan.range_min || range > scan.range_max) 
            continue;
        
        
        double angle = scan.angle_min + i * angle_increment;
        Point2D world_point = transformPoint(range, angle, robot_pose);
        world_points.push_back(world_point);
    }
    
    return world_points;
}

//Transform a single LIDAR point from robot frame to world frame
//TODO turn into more efficient matrix operation
Point2D LidarProcessor::transformPoint(double range, double angle, const Pose2D& pose) {
    double x_robot = range * std::cos(angle);
    double y_robot = range * std::sin(angle);
    
    double cos_theta = std::cos(pose.theta);
    double sin_theta = std::sin(pose.theta);
    
    Point2D world_point;
    world_point.x = pose.x + x_robot * cos_theta - y_robot * sin_theta;
    world_point.y = pose.y + x_robot * sin_theta + y_robot * cos_theta;
    
    return world_point;
}

} // namespace slam
