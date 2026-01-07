#include "zmq_bridge.h"
#include <iostream>
#include <csignal>

// Global flag for clean shutdown
volatile sig_atomic_t running = 1;

void signalHandler(int signal) {
    std::cout << "\n[Main] Caught signal " << signal << ", shutting down..." << std::endl;
    running = 0;
}

void messageCallback(const std::string& topic, const std::string& message) {
    std::cout << "[Main] Received on topic '" << topic << "': " << message << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "=== SLAM Core - ZMQ Subscriber ===" << std::endl;
    
    // Setup signal handling for clean shutdown (Ctrl+C)
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    try {
        slam::ZMQSubscriber subscriber("tcp://localhost:5555");
    
        subscriber.subscribe("hello");
        std::cout << "[Main] Waiting for messages... (Press Ctrl+C to exit)" << std::endl;
        
        // Listen for messages
        while (running) {
            subscriber.receiveMessage(messageCallback, 1000); // 1 second timeout
        }
        
        std::cout << "[Main] Exiting gracefully" << std::endl;
        
     } catch (const std::exception& e) {
        std::cerr << "[Main] Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
