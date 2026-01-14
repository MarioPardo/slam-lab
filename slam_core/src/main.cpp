#include "zmq_bridge.h"
#include "odometry.h"
#include "types.h"
#include <iostream>
#include <csignal>
#include <sstream>
#include <cmath>

// Global flag for clean shutdown
volatile sig_atomic_t running = 1;

void signalHandler(int signal) {
    std::cout << "\n[Main] Caught signal " << signal << ", shutting down..." << std::endl;
    running = 0;
}

// Helper function to parse JSON string
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

int main(int argc, char* argv[]) {
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
        
        subscriber.subscribe("robot_state");
        std::cout << "[Main] Waiting for messages... (Press Ctrl+C to exit)" << std::endl;
        
        int message_count = 0;
        
        // Message callback that sends a response
        auto messageCallback = [&publisher, &odometry, &message_count](const std::string& topic, const std::string& message) 
        {
            message_count++;
            
            // Parse odometry data from JSON
            slam::OdometryData odom;
            
            try {
                // Extract timestamp 
                size_t header_pos = message.find("\"header\"");
                if (header_pos != std::string::npos) {
                    odom.timestamp = extractDouble(message, "timestamp");
                }
                
                // Extract odometry values
                size_t odom_pos = message.find("\"odometry\"");
                if (odom_pos != std::string::npos) {
                    odom.left_encoder = extractDouble(message, "left_encoder");
                    odom.right_encoder = extractDouble(message, "right_encoder");
                    odom.gyro_z = extractDouble(message, "imu_gyro_z");
                    odom.compass_heading = extractDouble(message, "compass_heading");
                }
                
                slam::Pose2D pose = odometry.update(odom);
                
                // Print pose every 10 messages
                if (message_count % 10 == 0) {
                    std::cout << "[Main] Message " << message_count 
                              << " | Pose: x=" << pose.x 
                              << " m, y=" << pose.y 
                              << " m, θ=" << (pose.theta * 180.0 / M_PI) 
                              << "°" << std::endl;
                }
                
                // respond back with current pose
                std::ostringstream response;
                response << "{\"status\": \"ok\", "
                         << "\"message_count\": " << message_count << ", "
                         << "\"pose\": {\"x\": " << pose.x << ", "
                         << "\"y\": " << pose.y << ", "
                         << "\"theta\": " << pose.theta << "}}";
                
                publisher.publishMessage("response", response.str());
                
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
