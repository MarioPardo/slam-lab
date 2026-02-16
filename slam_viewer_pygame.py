#!/usr/bin/env python3
# Made using LLM
"""
SLAM Pygame Visualization Tool
Visualizes odometry vs ICP-corrected trajectories and lidar scans
"""

import pygame
import zmq
import json
import math
import sys
from collections import deque

class SLAMViewer:
    def __init__(self, width=1200, height=900, scale=150.0):
        """
        Initialize the SLAM Viewer
        
        Args:
            width: Window width in pixels
            height: Window height in pixels
            scale: Pixels per meter (zoom level)
        """
        pygame.init()
        
        # Display setup
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("SLAM Viewer - Odometry (Blue) vs ICP (Green)")
        
        # Coordinate transformation
        self.scale = scale  # pixels per meter (150 = 3x more zoom than default)
        self.origin_x = width // 2  # Center of screen (pixels)
        self.origin_y = height // 2
        
        # Colors
        self.COLOR_BG = (0, 0, 0)  # Black
        self.COLOR_ALL_LIDAR = (60, 60, 60)  # Dark gray (all accumulated lidar)
        self.COLOR_GLOBAL_MAP = (100, 100, 100)  # Medium gray (keyframes)
        self.COLOR_ODOM_TRAJ = (0, 100, 255)  # Blue
        self.COLOR_ICP_TRAJ = (0, 255, 100)  # Green
        self.COLOR_CURRENT_SCAN = (255, 50, 50)  # Red
        self.COLOR_TEXT = (255, 255, 255)  # White
        self.COLOR_GRID = (40, 40, 40)  # Very dark gray
        
        # Data storage
        self.odom_trajectory = []  # List of (x, y) in world coords
        self.icp_trajectory = []   # List of (x, y) in world coords
        self.current_scan = []     # List of (x, y) in world coords (most recent scan)
        self.all_lidar_points = [] # ALL lidar points ever received (occupancy grid style)
        self.global_map = []       # Historical lidar points (keyframes only)
        
        # Keyframe logic
        self.last_keyframe_x = 0.0
        self.last_keyframe_y = 0.0
        self.last_keyframe_theta = 0.0
        self.keyframe_dist_threshold = 0.5  # meters
        self.keyframe_angle_threshold = math.radians(10)  # radians
        
        # Camera pan/zoom
        self.camera_offset_x = 0  # Additional pan offset
        self.camera_offset_y = 0
        self.auto_center = True  # Follow robot automatically
        
        # Font for text
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 18)
        
        # Stats
        self.message_count = 0
        self.keyframe_count = 0
        
    def world_to_screen(self, x, y):
        """
        Convert world coordinates (meters) to screen coordinates (pixels)
        
        World coords: X=right, Y=forward (standard robotics)
        Screen coords: X=right, Y=down (pygame standard)
        
        Args:
            x: World X coordinate (meters, positive = right)
            y: World Y coordinate (meters, positive = forward)
            
        Returns:
            (screen_x, screen_y) tuple in pixels
        """
        # Apply scale
        screen_x = x * self.scale
        screen_y = -y * self.scale  # Flip Y axis (world Y up -> screen Y down)
        
        # Apply origin offset (center of screen)
        screen_x += self.origin_x + self.camera_offset_x
        screen_y += self.origin_y + self.camera_offset_y
        
        return (int(screen_x), int(screen_y))
    
    def screen_to_world(self, screen_x, screen_y):
        """Convert screen coordinates back to world coordinates (for debugging)"""
        x = (screen_x - self.origin_x - self.camera_offset_x) / self.scale
        y = -(screen_y - self.origin_y - self.camera_offset_y) / self.scale
        return (x, y)
    
    def should_add_keyframe(self, x, y, theta=0.0):
        """
        Determine if current pose should be added as keyframe
        
        Args:
            x, y: Current position in world coords
            theta: Current orientation (optional)
            
        Returns:
            bool: True if significant motion since last keyframe
        """
        dist = math.sqrt((x - self.last_keyframe_x)**2 + (y - self.last_keyframe_y)**2)
        angle_diff = abs(theta - self.last_keyframe_theta)
        
        # Normalize angle difference to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        angle_diff = abs(angle_diff)
        
        return (dist > self.keyframe_dist_threshold or 
                angle_diff > self.keyframe_angle_threshold)
    
    def update_data(self, odom_traj, icp_traj, lidar_points):
        """
        Update visualization data from SLAM system
        
        Args:
            odom_traj: List of dicts with 'x', 'y' keys (odometry trajectory)
            icp_traj: List of dicts with 'x', 'y' keys (ICP trajectory)
            lidar_points: List of dicts with 'x', 'y' keys (current scan in world frame)
        """
        self.message_count += 1
        
        # Update trajectories
        self.odom_trajectory = [(p['x'], p['y']) for p in odom_traj]
        self.icp_trajectory = [(p['x'], p['y']) for p in icp_traj]
        
        # Update current scan (most recent)
        self.current_scan = [(p['x'], p['y']) for p in lidar_points]
        
        # Accumulate ALL lidar points (occupancy grid style)
        self.all_lidar_points.extend(self.current_scan)
        
        # Limit total lidar points to prevent memory issues
        max_total_points = 100000  # 100k points max
        if len(self.all_lidar_points) > max_total_points:
            # Remove oldest points
            self.all_lidar_points = self.all_lidar_points[-max_total_points:]
        
        # Auto-center camera on latest ICP pose
        if self.auto_center and len(self.icp_trajectory) > 0:
            latest_x, latest_y = self.icp_trajectory[-1]
            # Don't update camera offset, just update origin
            # This keeps the robot roughly centered
            
        # Update global map with keyframes
        if len(self.icp_trajectory) > 0:
            latest_x, latest_y = self.icp_trajectory[-1]
            theta = 0.0  # We don't have theta in the data, so use distance only
            
            if self.should_add_keyframe(latest_x, latest_y, theta):
                # Add current scan to global map
                self.global_map.extend(self.current_scan)
                
                # Update keyframe reference
                self.last_keyframe_x = latest_x
                self.last_keyframe_y = latest_y
                self.last_keyframe_theta = theta
                self.keyframe_count += 1
                
                # Limit global map size to prevent memory issues
                max_map_points = 50000
                if len(self.global_map) > max_map_points:
                    self.global_map = self.global_map[-max_map_points:]
    
    def draw_grid(self):
        """Draw a reference grid (1 meter squares)"""
        # Draw vertical lines
        for i in range(-20, 21):
            x_world = i * 1.0  # 1 meter spacing
            x1, y1 = self.world_to_screen(x_world, -20)
            x2, y2 = self.world_to_screen(x_world, 20)
            pygame.draw.line(self.screen, self.COLOR_GRID, (x1, y1), (x2, y2), 1)
        
        # Draw horizontal lines
        for i in range(-20, 21):
            y_world = i * 1.0
            x1, y1 = self.world_to_screen(-20, y_world)
            x2, y2 = self.world_to_screen(20, y_world)
            pygame.draw.line(self.screen, self.COLOR_GRID, (x1, y1), (x2, y2), 1)
        
        # Draw axes (thicker)
        # X-axis (red)
        x1, y1 = self.world_to_screen(-20, 0)
        x2, y2 = self.world_to_screen(20, 0)
        pygame.draw.line(self.screen, (100, 0, 0), (x1, y1), (x2, y2), 2)
        
        # Y-axis (green)
        x1, y1 = self.world_to_screen(0, -20)
        x2, y2 = self.world_to_screen(0, 20)
        pygame.draw.line(self.screen, (0, 100, 0), (x1, y1), (x2, y2), 2)
    
    def draw_trajectory(self, trajectory, color, thickness=2):
        """Draw a trajectory as connected line segments"""
        if len(trajectory) < 2:
            return
        
        points = [self.world_to_screen(x, y) for x, y in trajectory]
        
        # Filter out off-screen points to prevent line drawing issues
        visible_points = [p for p in points if 0 <= p[0] < self.width and 0 <= p[1] < self.height]
        
        if len(visible_points) >= 2:
            pygame.draw.lines(self.screen, color, False, points, thickness)
    
    def draw_points(self, points, color, radius=2):
        """Draw a list of points as circles"""
        for x, y in points:
            screen_x, screen_y = self.world_to_screen(x, y)
            
            # Only draw if on screen
            if 0 <= screen_x < self.width and 0 <= screen_y < self.height:
                pygame.draw.circle(self.screen, color, (screen_x, screen_y), radius)
    
    def draw_stats(self):
        """Draw statistics overlay"""
        stats = [
            f"Messages: {self.message_count}",
            f"Keyframes: {self.keyframe_count}",
            f"Odom points: {len(self.odom_trajectory)}",
            f"ICP points: {len(self.icp_trajectory)}",
            f"All lidar pts: {len(self.all_lidar_points)}",
            f"Map points: {len(self.global_map)}",
            f"Scan points: {len(self.current_scan)}",
            f"Scale: {self.scale:.1f} px/m",
            "",
            "Controls:",
            "  +/- : Zoom",
            "  Arrows: Pan",
            "  C: Toggle auto-center",
            "  R: Reset view",
            "  Q/ESC: Quit"
        ]
        
        y = 10
        for line in stats:
            if line:  # Non-empty line
                text = self.small_font.render(line, True, self.COLOR_TEXT)
            else:  # Empty line for spacing
                y += 5
                continue
            
            # Draw background for readability
            bg_rect = text.get_rect()
            bg_rect.topleft = (10, y)
            bg_rect.inflate_ip(4, 2)
            pygame.draw.rect(self.screen, (0, 0, 0, 128), bg_rect)
            
            self.screen.blit(text, (10, y))
            y += 20
        
        # Draw legend at bottom
        legend_y = self.height - 80
        legend_items = [
            ("Odometry (Blue)", self.COLOR_ODOM_TRAJ),
            ("ICP Corrected (Green)", self.COLOR_ICP_TRAJ),
            ("Current Scan (Red)", self.COLOR_CURRENT_SCAN),
            ("Occupancy Grid (Dark Gray)", self.COLOR_ALL_LIDAR)
        ]
        
        for i, (label, color) in enumerate(legend_items):
            y = legend_y + i * 20
            # Draw color box
            pygame.draw.rect(self.screen, color, (10, y, 15, 15))
            # Draw label
            text = self.small_font.render(label, True, self.COLOR_TEXT)
            self.screen.blit(text, (30, y))
    
    def render(self):
        """Render all layers"""
        # Layer 1: Background
        self.screen.fill(self.COLOR_BG)
        
        # Layer 2: Grid
        self.draw_grid()
        
        # Layer 3: All accumulated lidar points (occupancy grid style - darkest)
        self.draw_points(self.all_lidar_points, self.COLOR_ALL_LIDAR, radius=1)
        
        # Layer 4: Global Map keyframes (medium gray)
        # self.draw_points(self.global_map, self.COLOR_GLOBAL_MAP, radius=1)
        
        # Layer 5: Odometry Trajectory (Blue)
        self.draw_trajectory(self.odom_trajectory, self.COLOR_ODOM_TRAJ, thickness=2)
        
        # Layer 6: ICP Trajectory (Green)
        self.draw_trajectory(self.icp_trajectory, self.COLOR_ICP_TRAJ, thickness=3)
        
        # Layer 7: Current Scan (Red, on top)
        self.draw_points(self.current_scan, self.COLOR_CURRENT_SCAN, radius=3)
        
        # Layer 8: Stats overlay
        self.draw_stats()
        
        # Update display
        pygame.display.flip()
    
    def handle_input(self):
        """Handle keyboard and mouse input, return False to quit"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                    return False
                
                # Zoom in/out
                elif event.key == pygame.K_EQUALS or event.key == pygame.K_PLUS:
                    self.scale *= 1.2
                    print(f"Zoom in: {self.scale:.1f} px/m")
                
                elif event.key == pygame.K_MINUS:
                    self.scale /= 1.2
                    print(f"Zoom out: {self.scale:.1f} px/m")
                
                # Pan camera
                elif event.key == pygame.K_LEFT:
                    self.camera_offset_x += 50
                    self.auto_center = False
                
                elif event.key == pygame.K_RIGHT:
                    self.camera_offset_x -= 50
                    self.auto_center = False
                
                elif event.key == pygame.K_UP:
                    self.camera_offset_y += 50
                    self.auto_center = False
                
                elif event.key == pygame.K_DOWN:
                    self.camera_offset_y -= 50
                    self.auto_center = False
                
                # Toggle auto-center
                elif event.key == pygame.K_c:
                    self.auto_center = not self.auto_center
                    print(f"Auto-center: {self.auto_center}")
                
                # Reset view
                elif event.key == pygame.K_r:
                    self.scale = 50.0
                    self.camera_offset_x = 0
                    self.camera_offset_y = 0
                    self.auto_center = True
                    print("View reset")
        
        return True


def main():
    """Main loop: receive ZMQ data and visualize"""
    print("=== SLAM Pygame Viewer ===")
    print("Connecting to SLAM core on tcp://localhost:5556...")
    
    # Setup ZMQ subscriber
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.connect("tcp://localhost:5556")
    subscriber.subscribe(b"visualization")  # Use bytes for topic
    subscriber.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
    
    print(f"ZMQ Socket created and subscribed to 'visualization' topic")
    
    # Create viewer
    viewer = SLAMViewer(width=1400, height=1000, scale=50.0)
    clock = pygame.time.Clock()
    
    print("Viewer running. Press Q or ESC to quit.")
    print("Waiting for SLAM data...")
    print("(Make sure slam_subscriber is running in another terminal!)")
    
    running = True
    frame_count = 0
    last_data_time = 0
    
    while running:
        # Handle input
        running = viewer.handle_input()
        
        # Try to receive data from ZMQ (non-blocking)
        try:
            # Receive multipart message [topic, data]
            msg_parts = subscriber.recv_multipart(flags=zmq.NOBLOCK)
            
            # Parse topic and message
            if len(msg_parts) >= 2:
                topic = msg_parts[0].decode('utf-8')
                message = msg_parts[1].decode('utf-8')
            else:
                # Fallback: single part with topic prefix
                full_msg = msg_parts[0].decode('utf-8')
                parts = full_msg.split(' ', 1)
                if len(parts) == 2:
                    topic = parts[0]
                    message = parts[1]
                else:
                    topic = "unknown"
                    message = full_msg
            
            if frame_count % 10 == 0:  # Print every 10th message
                print(f"Received message #{viewer.message_count + 1} (topic: {topic})")
            
            # Parse JSON data
            data = json.loads(message)
            
            odom_traj = data.get('odom_trajectory', [])
            icp_traj = data.get('icp_trajectory', [])
            lidar_points = data.get('lidar_points', [])
            
            # Update viewer
            viewer.update_data(odom_traj, icp_traj, lidar_points)
            last_data_time = frame_count
            
        except zmq.Again:
            # No data available
            if frame_count % 300 == 0 and frame_count > 0:  # Every 10 seconds at 30fps
                if frame_count - last_data_time > 300:
                    print(f"[WARNING] No data received for {(frame_count - last_data_time) // 30} seconds")
                    print("  Check if slam_subscriber is running and publishing data")
        except json.JSONDecodeError as e:
            print(f"JSON decode error: {e}")
            if 'message' in locals():
                print(f"Raw message: {message[:200]}...")  # Print first 200 chars
        except Exception as e:
            print(f"Error receiving data: {e}")
            import traceback
            traceback.print_exc()
        
        # Render frame
        viewer.render()
        
        # Cap framerate
        clock.tick(30)  # 30 FPS
        frame_count += 1
    
    print(f"\nShutting down after {frame_count} frames")
    subscriber.close()
    context.term()
    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
