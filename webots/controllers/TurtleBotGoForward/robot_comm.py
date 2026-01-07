"""ZMQ communication module for robot data publishing."""

import zmq
import json
import time


class RobotPublisher:
    """Publisher for sending robot sensor and state data via ZMQ."""
    
    def __init__(self, address="tcp://localhost:5555"):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(address)
        print(f"[RobotPublisher] Bound to {address}")
        
        # Small delay to allow subscribers to connect
        time.sleep(0.1)
    
    def publish_message(self, topic, message_dict):

        # Add timestamp if not present
        if 'timestamp' not in message_dict:
            message_dict['timestamp'] = time.time()
        
        # Serialize to JSON
        message_json = json.dumps(message_dict)
        
        # Send as topic + space + message
        full_message = f"{topic} {message_json}"
        self.socket.send_string(full_message)
    
    def publish_hello(self, message="Hello from robot!"):
        self.publish_message("hello", {"message": message})
    
    def close(self):
        """Clean up ZMQ resources."""
        self.socket.close()
        self.context.term()
        print("[RobotPublisher] Closed")
