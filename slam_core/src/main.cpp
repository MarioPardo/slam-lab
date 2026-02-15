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
    const std::vector<slam::Pose2D>& trajectory,
    const std::vector<std::pair<slam::Point2D, double>>& occupied_cells) {
    
    std::ostringstream viz_data;
    viz_data << "{\"trajectory\": [";
    for (size_t i = 0; i < trajectory.size(); i++) {
        if (i > 0) viz_data << ",";
        viz_data << "{\"x\":" << trajectory[i].x << ",\"y\":" << trajectory[i].y << "}";
    }
    viz_data << "],\"grid_cells\":[";
    for (size_t i = 0; i < occupied_cells.size(); i++) {
        if (i > 0) viz_data << ",";
        viz_data << "{\"x\":" << occupied_cells[i].first.x 
                 << ",\"y\":" << occupied_cells[i].first.y 
                 << ",\"p\":" << occupied_cells[i].second << "}";
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
        slam::OdometryProcessor odometry(0.033, 0.16);
        slam::LidarProcessor lidar;
        
        slam::OccupancyGrid grid(0.05, 400, 400, -10.0, -10.0);
        constexpr double occupied_threshold = 0.65;
        
        std::vector<slam::Pose2D> trajectory;
        slam::Pose2D prev_pose = {0.0, 0.0, 0.0};
        
        subscriber.subscribe("robot_state");
        std::cout << "[Main] Waiting for messages... (Press Ctrl+C to exit)" << std::endl;
        
        int message_count = 0;
        
        // Message callback that sends a response
        auto messageCallback = [&publisher, &odometry, &grid, &trajectory, &message_count, occupied_threshold](const std::string& /*topic*/, const std::string& message) 
        {
            message_count++;
            
            try {
                slam::OdometryData odom = parseOdometryData(message);
                slam::Pose2D pose = odometry.update(odom);
                trajectory.push_back(pose);
                
                slam::LidarScan scan = parseLidarScan(message, odom.timestamp);
                if (!scan.ranges.empty()) {
                    grid.updateWithScan(scan, pose);
                }
                
                if (message_count % 1 == 0) {
                    auto occupied_cells = grid.getOccupiedWorldPoints(occupied_threshold);
                    
                    std::cout << "[Main] Message " << message_count 
                              << " | Pose: (" << pose.x << ", " << pose.y << ", " 
                              << (pose.theta * 180.0 / M_PI) << "°)"
                              << " | Trajectory: " << trajectory.size()
                              << " | Occupied Cells: " << occupied_cells.size() << std::endl;
                    
                    std::string viz_message = createVisualizationMessage(trajectory, occupied_cells);
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
