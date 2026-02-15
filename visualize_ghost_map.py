import zmq
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5556")
socket.subscribe("visualization")

print("[Visualizer] Connected to C++ publisher on port 5556")
print("[Visualizer] Waiting for visualization data...")

trajectory_x = []
trajectory_y = []
lidar_points_x = []
lidar_points_y = []

fig, ax = plt.subplots(figsize=(12, 10))
ax.set_aspect('equal', 'box')
ax.grid(True, alpha=0.3)
ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_title('Robot Trajectory + LiDAR Scan Points')

trajectory_line, = ax.plot([], [], 'b-', linewidth=2, label='Robot Path', zorder=3)
robot_marker, = ax.plot([], [], 'ro', markersize=10, label='Current Position', zorder=4)
lidar_scatter = ax.scatter([], [], c='red', s=5, alpha=0.5, label='LiDAR Points', zorder=2)

ax.legend()

def update_plot(frame):
    global trajectory_x, trajectory_y, lidar_points_x, lidar_points_y
    
    socket.setsockopt(zmq.RCVTIMEO, 100)
    
    try:
        full_message = socket.recv_string()
        message_parts = full_message.split(' ', 1)
        
        if len(message_parts) != 2:
            return trajectory_line, robot_marker, lidar_scatter
        
        data = json.loads(message_parts[1])
        
        trajectory = data.get('trajectory', [])
        lidar_points = data.get('lidar_points', [])
        
        trajectory_x = [p['x'] for p in trajectory]
        trajectory_y = [p['y'] for p in trajectory]
        
        lidar_points_x = [p['x'] for p in lidar_points]
        lidar_points_y = [p['y'] for p in lidar_points]
        
        # Draw trajectory line
        if trajectory_x and trajectory_y:
            trajectory_line.set_data(trajectory_x, trajectory_y)
            # Show current position
            robot_marker.set_data([trajectory_x[-1]], [trajectory_y[-1]])
        
        # Update LiDAR points scatter plot
        if lidar_points_x and lidar_points_y:
            lidar_scatter.set_offsets(list(zip(lidar_points_x, lidar_points_y)))
        else:
            lidar_scatter.set_offsets([])

        # Adjust plot limits dynamically
        if trajectory_x or lidar_points_x:
            all_x = trajectory_x + lidar_points_x
            all_y = trajectory_y + lidar_points_y
            if all_x and all_y:
                margin = 0.5
                ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
                ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        
        print(f"[Visualizer] Updated: {len(trajectory_x)} trajectory points, {len(lidar_points_x)} LiDAR points")
        
    except zmq.Again:
        pass
    except Exception as e:
        print(f"[Visualizer] Error: {e}")
    
    return trajectory_line, robot_marker, lidar_scatter

ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=True, cache_frame_data=False)

plt.show()

socket.close()
context.term()
print("[Visualizer] Closed")
