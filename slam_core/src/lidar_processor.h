#ifndef SLAM_LIDAR_PROCESSOR_H
#define SLAM_LIDAR_PROCESSOR_H

#include "types.h"
#include <vector>

namespace slam {

class LidarProcessor {
public:
    LidarProcessor();
    
    std::vector<Point2D> transformToWorld(const LidarScan& scan, const Pose2D& robot_pose);
    
private:
    Point2D transformPoint(double range, double angle, const Pose2D& pose);
};

} // namespace slam

#endif // SLAM_LIDAR_PROCESSOR_H
