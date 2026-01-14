#ifndef SLAM_ODOMETRY_H
#define SLAM_ODOMETRY_H

#include "types.h"

namespace slam {

class OdometryProcessor {
public:

    OdometryProcessor(double wheel_radius, double wheelbase);
    
    Pose2D update(const OdometryData& odom);
    
    Pose2D getCurrentPose() const;
    
    void reset(const Pose2D& pose = Pose2D{0, 0, 0});
    
    void initialize(const OdometryData& odom, const Pose2D& initial_pose = Pose2D{0, 0, 0});
    
    bool isInitialized() const { return initialized_; }

private:
    // Robot physical parameters
    double wheel_radius_;   
    double wheelbase_;      
    
    // Current state
    Pose2D current_pose_;
    double prev_left_encoder_theta;   // radians
    double prev_right_encoder_theta;  // radians
    bool initialized_;
    

    double normalizeAngle(double angle) const;
};

} 

#endif // SLAM_ODOMETRY_H
