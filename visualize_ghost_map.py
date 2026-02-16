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

odom_trajectory_x = []
odom_trajectory_y = []
icp_trajectory_x = []
icp_trajectory_y = []
lidar_points_x = []
lidar_points_y = []

fig, ax = plt.subplots(figsize=(12, 10))
ax.set_aspect('equal', 'box')
ax.grid(True, alpha=0.3)
ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_title('SLAM Comparison: Odometry vs ICP-Corrected')

odom_line, = ax.plot([], [], 'b--', linewidth=1.5, alpha=0.7, label='Odometry Only', zorder=3)
icp_line, = ax.plot([], [], 'g-', linewidth=2, label='ICP Corrected', zorder=4)
odom_marker, = ax.plot([], [], 'bo', markersize=8, label='Odom Position', zorder=5)
icp_marker, = ax.plot([], [], 'go', markersize=10, label='ICP Position', zorder=6)
lidar_scatter = ax.scatter([], [], c='red', s=5, alpha=0.5, label='LiDAR Points', zorder=2)

ax.legend()

def update_plot(frame):
    global odom_trajectory_x, odom_trajectory_y, icp_trajectory_x, icp_trajectory_y, lidar_points_x, lidar_points_y
    
    socket.setsockopt(zmq.RCVTIMEO, 100)
    
    try:
        full_message = socket.recv_string()
        message_parts = full_message.split(' ', 1)
        
        if len(message_parts) != 2:
            return odom_line, icp_line, odom_marker, icp_marker, lidar_scatter
        
        data = json.loads(message_parts[1])
        
        odom_trajectory = data.get('odom_trajectory', [])
        icp_trajectory = data.get('icp_trajectory', [])
        lidar_points = data.get('lidar_points', [])
        
        odom_trajectory_x = [p['x'] for p in odom_trajectory]
        odom_trajectory_y = [p['y'] for p in odom_trajectory]
        
        icp_trajectory_x = [p['x'] for p in icp_trajectory]
        icp_trajectory_y = [p['y'] for p in icp_trajectory]
        
        lidar_points_x = [p['x'] for p in lidar_points]
        lidar_points_y = [p['y'] for p in lidar_points]
        
        # Draw odometry trajectory
        if odom_trajectory_x and odom_trajectory_y:
            odom_line.set_data(odom_trajectory_x, odom_trajectory_y)
            odom_marker.set_data([odom_trajectory_x[-1]], [odom_trajectory_y[-1]])
        
        # Draw ICP trajectory
        if icp_trajectory_x and icp_trajectory_y:
            icp_line.set_data(icp_trajectory_x, icp_trajectory_y)
            icp_marker.set_data([icp_trajectory_x[-1]], [icp_trajectory_y[-1]])
        
        # Update LiDAR points scatter plot
        if lidar_points_x and lidar_points_y:
            offsets = np.column_stack((lidar_points_x, lidar_points_y))
            lidar_scatter.set_offsets(offsets)
        else:
            lidar_scatter.set_offsets(np.empty((0, 2)))

        # Adjust plot limits dynamically
        all_x = odom_trajectory_x + icp_trajectory_x + lidar_points_x
        all_y = odom_trajectory_y + icp_trajectory_y + lidar_points_y
        if all_x and all_y:
            margin = 0.5
            ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
            ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        
        print(f"[Visualizer] Odom: {len(odom_trajectory_x)}, ICP: {len(icp_trajectory_x)}, LiDAR: {len(lidar_points_x)}")
        
    except zmq.Again:
        pass
    except Exception as e:
        print(f"[Visualizer] Error: {e}")
    
    return odom_line, icp_line, odom_marker, icp_marker, lidar_scatter

ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=True, cache_frame_data=False)

plt.show()

socket.close()
context.term()
print("[Visualizer] Closed")
