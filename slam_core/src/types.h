#ifndef SLAM_TYPES_H
#define SLAM_TYPES_H

#include <vector>

namespace slam {

struct Pose2D {
    double x;     
    double y;     
    double theta;  
    
    Pose2D() : x(0.0), y(0.0), theta(0.0) {}
    Pose2D(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
};

struct OdometryData {
    double timestamp;         
    double left_encoder;      
    double right_encoder;    
    double gyro_z;            // rad/s 
    double compass_heading;   // radians (absolute heading)
    
    OdometryData() : timestamp(0.0), left_encoder(0.0), right_encoder(0.0), 
                     gyro_z(0.0), compass_heading(0.0) {}
};


struct LidarScan {
    double timestamp;           
    std::vector<double> ranges; 
    int count;                  // number of rays
    double angle_min;           
    double angle_max;           
    double range_min;           
    double range_max;          
    
    LidarScan() : timestamp(0.0), count(0), angle_min(0.0), 
                  angle_max(0.0), range_min(0.0), range_max(0.0) {}
};

struct Point2D {
    double x; 
    double y; 
    
    Point2D() : x(0.0), y(0.0) {}
    Point2D(double x_, double y_) : x(x_), y(y_) {}
};

} // namespace slam

#endif // SLAM_TYPES_H
