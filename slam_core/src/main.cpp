#include "zmq_bridge.h"
#include "odometry.h"
#include "lidar_processor.h"
#include "occupancy_grid.h"
#include "icp_matcher.h"
#include "types.h"
#include <iostream>
#include <csignal>
#include <sstream>
#include <cmath>
#include <vector>

// Global flag for clean shutdown
volatile sig_atomic_t running = 1;

void signalHandler(int signal) {
    std::cout << "\n[Main] Caught signal " << signal << ", shutting down..." << std::endl;
    running = 0;
}

// Helper function to parse JSON string////

double extractDouble(const std::string& json, const std::string& key) {
    size_t pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return 0.0;
    
    pos = json.find(":", pos);
    if (pos == std::string::npos) return 0.0;
    
    size_t start = pos + 1;
    size_t end = json.find_first_of(",}", start);
    
    std::string value = json.substr(start, end - start);
    return std::stod(value);
}

int extractInt(const std::string& json, const std::string& key) {
    size_t pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return 0;
    
    pos = json.find(":", pos);
    if (pos == std::string::npos) return 0;
    
    size_t start = pos + 1;
    size_t end = json.find_first_of(",}", start);
    
    std::string value = json.substr(start, end - start);
    return std::stoi(value);
}

std::vector<double> extractDoubleArray(const std::string& json, const std::string& key) {
    std::vector<double> result;
    size_t pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return result;
    
    size_t array_start = json.find("[", pos);
    size_t array_end = json.find("]", array_start);
    if (array_start == std::string::npos || array_end == std::string::npos) return result;
    
    std::string array_content = json.substr(array_start + 1, array_end - array_start - 1);
    std::istringstream iss(array_content);
    std::string token;
    
    while (std::getline(iss, token, ',')) {
        try {
            result.push_back(std::stod(token));
        } catch (...) {
            result.push_back(std::numeric_limits<double>::infinity());
        }
    }
    
    return result;
}

slam::OdometryData parseOdometryData(const std::string& message) {
    slam::OdometryData odom;
    
    size_t header_pos = message.find("\"header\"");
    if (header_pos != std::string::npos) {
        odom.timestamp = extractDouble(message, "timestamp");
    }
    
    size_t odom_pos = message.find("\"odometry\"");
    if (odom_pos != std::string::npos) {
        odom.left_encoder = extractDouble(message, "left_encoder");
        odom.right_encoder = extractDouble(message, "right_encoder");
        odom.gyro_z = extractDouble(message, "imu_gyro_z");
        odom.compass_heading = extractDouble(message, "compass_heading");
    }
    
    return odom;
}

slam::LidarScan parseLidarScan(const std::string& message, double timestamp) {
    slam::LidarScan scan;
    
    size_t lidar_pos = message.find("\"lidar\"");
    if (lidar_pos != std::string::npos) {
        scan.timestamp = timestamp;
        scan.count = extractInt(message, "count");
        scan.angle_min = extractDouble(message, "angle_min");
        scan.angle_max = extractDouble(message, "angle_max");
        scan.range_min = extractDouble(message, "range_min");
        scan.range_max = extractDouble(message, "range_max");
        scan.ranges = extractDoubleArray(message, "ranges");
    }
    
    return scan;
}

std::string createVisualizationMessage(
    const std::vector<slam::Pose2D>& odom_trajectory,
    const std::vector<slam::Pose2D>& icp_trajectory,
    const std::vector<slam::Point2D>& lidar_points) {
    
    std::ostringstream viz_data;
    viz_data << "{\"odom_trajectory\": [";
    for (size_t i = 0; i < odom_trajectory.size(); i++) {
        if (i > 0) viz_data << ",";
        viz_data << "{\"x\":" << odom_trajectory[i].x << ",\"y\":" << odom_trajectory[i].y << "}";
    }
    viz_data << "],\"icp_trajectory\":[";
    for (size_t i = 0; i < icp_trajectory.size(); i++) {
        if (i > 0) viz_data << ",";
        viz_data << "{\"x\":" << icp_trajectory[i].x << ",\"y\":" << icp_trajectory[i].y << "}";
    }
    viz_data << "],\"lidar_points\":[";
    for (size_t i = 0; i < lidar_points.size(); i++) {
        if (i > 0) viz_data << ",";
        viz_data << "{\"x\":" << lidar_points[i].x 
                 << ",\"y\":" << lidar_points[i].y << "}";
    }
    viz_data << "]}";
    
    return viz_data.str();
}

///////

int main(int /*argc*/, char* /*argv*/[]) {
    std::cout << "=== SLAM Core - ZMQ Bidirectional Communication ===" << std::endl;
    
    // Setup signal handling for clean shutdown (Ctrl+C)
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    try {
        slam::ZMQSubscriber subscriber("tcp://localhost:5555");
        slam::ZMQPublisher publisher("tcp://*:5556");
        
        // TurtleBot parameters
        // Wheel radius: 33mm = 0.033m
        // Wheelbase: 160mm = 0.16m

        //Set up sensor processors
        slam::OdometryProcessor odometry(0.033, 0.16);
        slam::LidarProcessor lidar;
        
        //Set up variables tracking robot trajectory etc
        slam::OccupancyGrid grid(0.05, 400, 400, -10.0, -10.0);
        constexpr double occupied_threshold = 0.65;
        std::vector<slam::Pose2D> odom_trajectory;   // Odometry-only trajectory
        std::vector<slam::Pose2D> icp_trajectory;    // ICP-corrected trajectory

        slam::Pose2D prev_icp_pose = {0.0, 0.0, 0.0};
        slam::Pose2D prev_odom_estimate = {0,0,0};
        std::vector<Eigen::Vector2d> prev_point_cloud = {};
        bool first_scan = true;
        
        subscriber.subscribe("robot_state");
        std::cout << "[Main] Waiting for messages... (Press Ctrl+C to exit)" << std::endl;
        
        int message_count = 0;
        
        // Message callback that sends a response
        auto messageCallback = [&publisher, &odometry,&lidar, &grid, &odom_trajectory, &icp_trajectory, &message_count, &first_scan, &prev_icp_pose, &prev_odom_estimate, &prev_point_cloud, occupied_threshold](const std::string& /*topic*/, const std::string& message)
        {
            message_count++;
            
            try {
        
                slam::Pose2D curr_odom_pose = {0,0,0};
                slam::Pose2D curr_icp_pose = {0,0,0}; // This will track the "Fused" (Best) State

                // 1. Use Odometry
                slam::OdometryData odom = parseOdometryData(message);
                slam::Pose2D odom_estimate = odometry.update(odom);
                curr_odom_pose = odom_estimate;

                // 2. Use Lidar
                slam::LidarScan scan = parseLidarScan(message, odom.timestamp);
                std::vector<slam::Point2D> world_lidar_points;

                if (!scan.ranges.empty()) 
                {
                    //get point cloud from robot pov
                    std::vector<Eigen::Vector2d> curr_point_cloud = lidar.scanToPointCloud(scan);
                    
                    if (!first_scan && !prev_point_cloud.empty())
                    {
                        //initial guess
                        slam::Transform2D odom_delta = computePoseDelta(odom_estimate, prev_odom_estimate);
                        
                        slam::ICPResult icp_result = alignPointClouds(
                            prev_point_cloud, 
                            curr_point_cloud, 
                            odom_delta, 
                            100,    
                            1e-3,  
                            0.1   
                        );
                        
                        // Compare ICP's proposed motion vs Odometry's predicted motion
                        double dx = icp_result.transform.translation.x() - odom_delta.translation.x();
                        double dy = icp_result.transform.translation.y() - odom_delta.translation.y();
                        double icptheta = std::atan2(icp_result.transform.rotation(1,0), icp_result.transform.rotation(0,0));
                        double odomtheta = std::atan2(odom_delta.rotation(1,0), odom_delta.rotation(0,0));
                        double dtheta = std::abs(icptheta - odomtheta);
                        double trans_error = std::sqrt(dx*dx + dy*dy);

                        // Thresholds: e.g., 20cm difference or 10 degrees difference is "too much"
                        bool is_valid = icp_result.converged && (trans_error < 0.20) && (dtheta < (10.0 * M_PI / 180.0));

                        if (is_valid) {
                            curr_icp_pose = prev_icp_pose.transform(icp_result.transform);
                        } else {
                            curr_icp_pose = prev_icp_pose.transform(odom_delta);
                        }
                    }
                    else
                    {
                        // First scan initialization
                        curr_icp_pose = odom_estimate;
                        first_scan = false;
                    }
                    
                    // Update state for next iteration
                    prev_point_cloud = curr_point_cloud;
                    prev_odom_estimate = odom_estimate;
                    prev_icp_pose = curr_icp_pose; 
                    
                    // Visualize using the BEST estimate (Fused)
                    world_lidar_points = lidar.transformToWorld(scan, curr_odom_pose);
                }

                // Store history
                odom_trajectory.push_back(curr_odom_pose); // Pure Odom
                icp_trajectory.push_back(curr_icp_pose);   // Odom + ICP corrections
                
                //Create and Send Message to Visualizer
                if (message_count % 1 == 0) {
                    std::cout << "[Main] Message " << message_count 
                              << " | Odom: (" << curr_odom_pose.x << ", " << curr_odom_pose.y << ", " 
                              << (curr_odom_pose.theta * 180.0 / M_PI) << "°)"
                              << " | ICP: (" << curr_icp_pose.x << ", " << curr_icp_pose.y << ", "
                              << (curr_icp_pose.theta * 180.0 / M_PI) << "°)"
                              << " | Points: " << world_lidar_points.size() << std::endl;
                    
                    std::string viz_message = createVisualizationMessage(odom_trajectory, icp_trajectory, world_lidar_points);
                    publisher.publishMessage("visualization", viz_message);
                }
                
            } catch (const std::exception& e) {
                std::cerr << "[Main] Error parsing message: " << e.what() << std::endl;
            }
        };
        
        // Listen for messages
        while (running) {
            subscriber.receiveMessage(messageCallback, 1000); // 1 second timeout
        }
        
        std::cout << "[Main] Exiting gracefully" << std::endl;
        std::cout << "[Main] Total messages processed: " << message_count << std::endl;
        
     } catch (const std::exception& e) {
        std::cerr << "[Main] Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
