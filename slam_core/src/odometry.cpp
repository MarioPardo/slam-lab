#include "odometry.h"
#include <cmath>
#include <iostream>

namespace slam {

OdometryProcessor::OdometryProcessor(double wheel_radius, double wheelbase)
    : wheel_radius_(wheel_radius),
      wheelbase_(wheelbase),
      current_pose_(),
      prev_left_encoder_theta(0.0),
      prev_right_encoder_theta(0.0),
      initialized_(false)
{
    std::cout << "[OdometryProcessor] Initialized with:" << std::endl;
    std::cout << "  Wheel radius: " << wheel_radius_ << " m" << std::endl;
    std::cout << "  Wheelbase: " << wheelbase_ << " m" << std::endl;
}

void OdometryProcessor::initialize(const OdometryData& odom, const Pose2D& initial_pose)
{
    current_pose_ = initial_pose;
    prev_left_encoder_theta = odom.left_encoder;
    prev_right_encoder_theta = odom.right_encoder;
    initialized_ = true;
    
    std::cout << "[OdometryProcessor] Initialized at pose: (" 
              << current_pose_.x << ", " 
              << current_pose_.y << ", " 
              << current_pose_.theta << " rad)" << std::endl;
}

//update the pose given odometry dat
Pose2D OdometryProcessor::update(const OdometryData& odom)
{
    if (!initialized_) {
        initialize(odom);
        return current_pose_;
    }
    
    double delta_theta_left = odom.left_encoder - prev_left_encoder_theta;
    double delta_theta_right = odom.right_encoder - prev_right_encoder_theta;

    double distance_left = wheel_radius_ * delta_theta_left;
    double distance_right = wheel_radius_ * delta_theta_right;
    
    //Find change in robot pose
    double delta_s = (distance_left + distance_right) / 2.0;  
    double delta_theta = (distance_right - distance_left) / wheelbase_;  
    
  
    //Update Pose using differential drive kinematics
    if (std::abs(delta_theta) < 1e-6) // Straight motion
    {

        current_pose_.x += delta_s * std::cos(current_pose_.theta);
        current_pose_.y += delta_s * std::sin(current_pose_.theta);
    } else { // Arc motion
        double radius = delta_s / delta_theta;  // radius of the arc
        
        current_pose_.x += radius * (std::sin(current_pose_.theta + delta_theta) - std::sin(current_pose_.theta));
        current_pose_.y += radius * (-std::cos(current_pose_.theta + delta_theta) + std::cos(current_pose_.theta));
    }
    
    // Update orientation
    current_pose_.theta += delta_theta;
    current_pose_.theta = normalizeAngle(current_pose_.theta);
    
    // Store values for next iteration
    prev_left_encoder_theta = odom.left_encoder;
    prev_right_encoder_theta = odom.right_encoder;
    
    return current_pose_;
}

Pose2D OdometryProcessor::getCurrentPose() const
{
    return current_pose_;
}

void OdometryProcessor::reset(const Pose2D& pose)
{
    current_pose_ = pose;
    prev_left_encoder_theta = 0.0;
    prev_right_encoder_theta = 0.0;
    initialized_ = false;
    
    std::cout << "[OdometryProcessor] Reset to pose: (" 
              << current_pose_.x << ", " 
              << current_pose_.y << ", " 
              << current_pose_.theta << " rad)" << std::endl;
}

 // Normalize to [-pi, pi]
double OdometryProcessor::normalizeAngle(double angle) const
{
    while (angle > M_PI) 
    {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) 
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

} // namespace slam
