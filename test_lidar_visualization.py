#!/usr/bin/env python3
"""
Simple LiDAR visualization to verify angle mapping.
Shows LiDAR hit points in robot frame with the robot at origin.
"""

import zmq
import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5555")  # Python controller publishes here
socket.subscribe("robot_state")

print("[LiDAR Viz] Connected to Python controller on port 5555")
print("[LiDAR Viz] Waiting for robot_state messages...")
print("=" * 60)

def visualize_lidar_scan(lidar_data):
    """
    Visualize a single LiDAR scan in robot frame.
    Robot is at origin (0, 0) facing UP (+Y direction).
    """
    angle_min = lidar_data['angle_min']
    angle_max = lidar_data['angle_max']
    ranges = lidar_data['ranges']
    range_min = lidar_data['range_min']
    range_max = lidar_data['range_max']
    count = lidar_data['count']
    
    angle_increment = (angle_max - angle_min) / (count - 1)
    
    print(f"\n{'='*60}")
    print(f"LiDAR Scan Analysis:")
    print(f"  angle_min: {np.degrees(angle_min):.1f}°")
    print(f"  angle_max: {np.degrees(angle_max):.1f}°")
    print(f"  count: {count}")
    print(f"  angle_increment: {np.degrees(angle_increment):.2f}°")
    print(f"{'='*60}\n")
    
    # Count valid points
    valid_indices = []
    for idx, range_val in enumerate(ranges):
        if not (np.isinf(range_val) or np.isnan(range_val)):
            if range_min <= range_val <= range_max:
                valid_indices.append(idx)
    
    # Print first, middle, last VALID points
    print("Index Verification (valid points only):")
    if len(valid_indices) >= 3:
        for idx in [valid_indices[0], valid_indices[len(valid_indices)//2], valid_indices[-1]]:
            range_val = ranges[idx]
            # Test both interpretations
            angle_forward = angle_min + idx * angle_increment
            angle_reverse = angle_max - idx * angle_increment
            
            print(f"\nIndex {idx} (of {count}):")
            print(f"  Range: {range_val:.3f}m")
            print(f"  If forward (min→max): angle = {np.degrees(angle_forward):+.1f}°")
            print(f"  If reverse (max→min): angle = {np.degrees(angle_reverse):+.1f}°")
    
    # Create figure
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Test BOTH interpretations
    for ax, title, angle_func in [
        (ax1, "Interpretation 1: ranges[0] at angle_min (RIGHT)", 
         lambda i: angle_min + i * angle_increment),
        (ax2, "Interpretation 2: ranges[0] at angle_max (LEFT)", 
         lambda i: angle_max - i * angle_increment)
    ]:
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title(title, fontsize=10)
        ax.set_xlabel('X (meters) - RIGHT →')
        ax.set_ylabel('Y (meters) - FORWARD ↑')
        
        # Draw robot at origin
        robot_size = 0.1
        robot = patches.Rectangle((-robot_size/2, -robot_size/2), 
                                  robot_size, robot_size, 
                                  facecolor='blue', edgecolor='black', linewidth=2)
        ax.add_patch(robot)
        
        # Draw forward direction arrow
        ax.arrow(0, 0, 0, 0.3, head_width=0.05, head_length=0.05, 
                fc='blue', ec='blue', linewidth=2)
        ax.text(0.05, 0.35, 'FORWARD', fontsize=10, color='blue', weight='bold')
        
        # Draw LiDAR field of view
        ax.plot([0, 2*np.cos(angle_max)], [0, 2*np.sin(angle_max)], 
               'g--', alpha=0.5, linewidth=1, label='FOV edges')
        ax.plot([0, 2*np.cos(angle_min)], [0, 2*np.sin(angle_min)], 
               'g--', alpha=0.5, linewidth=1)
        
        # Add angle labels
        ax.text(2*np.cos(angle_max)*1.1, 2*np.sin(angle_max)*1.1, 
               f'{np.degrees(angle_max):.0f}°\nLEFT', 
               fontsize=8, color='green', ha='left')
        ax.text(2*np.cos(angle_min)*1.1, 2*np.sin(angle_min)*1.1, 
               f'{np.degrees(angle_min):.0f}°\nRIGHT', 
               fontsize=8, color='green', ha='right')
        
        # Plot all valid points
        points_x = []
        points_y = []
        first_valid_idx = None
        last_valid_idx = None
        
        for i, range_val in enumerate(ranges):
            if np.isinf(range_val) or np.isnan(range_val):
                continue
            if range_val < range_min or range_val > range_max:
                continue
            
            if first_valid_idx is None:
                first_valid_idx = i
            last_valid_idx = i
            
            angle = angle_func(i)
            x = range_val * np.cos(angle)
            y = range_val * np.sin(angle)
            
            points_x.append(x)
            points_y.append(y)
        
        # Plot points
        if points_x:
            ax.scatter(points_x, points_y, c='red', s=20, alpha=0.6, label='LiDAR hits')
            
            # Highlight first and last points
            if len(points_x) > 0:
                ax.scatter(points_x[0], points_y[0], c='green', s=150, 
                          marker='o', edgecolors='black', linewidth=2,
                          label=f'First valid (idx {first_valid_idx})', zorder=5)
            if len(points_x) > 1:
                ax.scatter(points_x[-1], points_y[-1], c='orange', s=150, 
                          marker='s', edgecolors='black', linewidth=2,
                          label=f'Last valid (idx {last_valid_idx})', zorder=5)
        
        ax.legend(loc='upper right', fontsize=8)
        ax.set_xlim(-2, 2)
        ax.set_ylim(-0.5, 2.5)
    
    plt.tight_layout()
    plt.show(block=True)

def main():
    socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout for faster updates
    
    # Set up interactive plotting
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    scan_count = 0
    
    try:
        while True:
            try:
                full_message = socket.recv_string()
                message_parts = full_message.split(' ', 1)
                
                if len(message_parts) != 2:
                    continue
                
                topic = message_parts[0]
                data = json.loads(message_parts[1])
                
                if 'lidar' in data:
                    scan_count += 1
                    
                    # Clear previous plots
                    ax1.clear()
                    ax2.clear()
                    
                    lidar_data = data['lidar']
                    angle_min = lidar_data['angle_min']
                    angle_max = lidar_data['angle_max']
                    ranges = lidar_data['ranges']
                    range_min = lidar_data['range_min']
                    range_max = lidar_data['range_max']
                    count = lidar_data['count']
                    
                    angle_increment = (angle_max - angle_min) / (count - 1)
                    
                    # Plot both interpretations
                    for ax, title, angle_func in [
                        (ax1, "Interpretation 1: ranges[0] at angle_min (RIGHT)", 
                         lambda i: angle_min + i * angle_increment),
                        (ax2, "Interpretation 2: ranges[0] at angle_max (LEFT)", 
                         lambda i: angle_max - i * angle_increment)
                    ]:
                        ax.set_aspect('equal')
                        ax.grid(True, alpha=0.3)
                        ax.set_title(f"{title}\nScan #{scan_count}", fontsize=9)
                        ax.set_xlabel('X (meters) - RIGHT →')
                        ax.set_ylabel('Y (meters) - FORWARD ↑')
                        
                        # Draw robot
                        robot_size = 0.1
                        robot = patches.Rectangle((-robot_size/2, -robot_size/2), 
                                                  robot_size, robot_size, 
                                                  facecolor='blue', edgecolor='black', linewidth=2)
                        ax.add_patch(robot)
                        
                        # Forward arrow
                        ax.arrow(0, 0, 0, 0.3, head_width=0.05, head_length=0.05, 
                                fc='blue', ec='blue', linewidth=2)
                        ax.text(0.05, 0.35, 'FWD', fontsize=9, color='blue', weight='bold')
                        
                        # FOV edges
                        ax.plot([0, 2*np.cos(angle_max)], [0, 2*np.sin(angle_max)], 
                               'g--', alpha=0.5, linewidth=1)
                        ax.plot([0, 2*np.cos(angle_min)], [0, 2*np.sin(angle_min)], 
                               'g--', alpha=0.5, linewidth=1)
                        
                        ax.text(2*np.cos(angle_max)*1.05, 2*np.sin(angle_max)*1.05, 
                               'LEFT\n+45°', fontsize=7, color='green', ha='left')
                        ax.text(2*np.cos(angle_min)*1.05, 2*np.sin(angle_min)*1.05, 
                               'RIGHT\n-45°', fontsize=7, color='green', ha='right')
                        
                        # Plot points
                        points_x = []
                        points_y = []
                        
                        for i, range_val in enumerate(ranges):
                            if np.isinf(range_val) or np.isnan(range_val):
                                continue
                            if range_val < range_min or range_val > range_max:
                                continue
                            
                            angle = angle_func(i)
                            x = range_val * np.cos(angle)
                            y = range_val * np.sin(angle)
                            
                            points_x.append(x)
                            points_y.append(y)
                        
                        if points_x:
                            ax.scatter(points_x, points_y, c='red', s=15, alpha=0.7)
                        
                        ax.set_xlim(-2, 2)
                        ax.set_ylim(-0.5, 2.5)
                    
                    plt.tight_layout()
                    plt.draw()
                    plt.pause(0.001)  # Small pause to update display
                    
                    print(f"\r[LiDAR Viz] Scan #{scan_count} - {len(points_x)} points", end='', flush=True)
                    
            except zmq.Again:
                plt.pause(0.01)  # Keep GUI responsive
                continue
                
    except KeyboardInterrupt:
        print("\n\n[LiDAR Viz] Exiting...")
    finally:
        plt.close('all')
        socket.close()
        context.term()

if __name__ == "__main__":
    main()

