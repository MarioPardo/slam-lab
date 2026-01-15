import zmq
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5556")
socket.subscribe("visualization")

print("[Visualizer] Connected to C++ publisher on port 5556")
print("[Visualizer] Waiting for visualization data...")

trajectory_x = []
trajectory_y = []
lidar_x = []
lidar_y = []

fig, ax = plt.subplots(figsize=(12, 10))
ax.set_aspect('equal', 'box')
ax.grid(True, alpha=0.3)
ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_title('Ghost Map: Odometry Trajectory + Lidar Points')

trajectory_line, = ax.plot([], [], 'r-', linewidth=2, label='Robot Path')
lidar_scatter = ax.scatter([], [], c='blue', s=1, alpha=0.3, label='Lidar Points')

ax.legend()

def update_plot(frame):
    global trajectory_x, trajectory_y, lidar_x, lidar_y
    
    socket.setsockopt(zmq.RCVTIMEO, 100)
    
    try:
        full_message = socket.recv_string()
        parts = full_message.split(' ', 1)
        
        if len(parts) != 2:
            return trajectory_line, lidar_scatter
        
        topic = parts[0]
        data = json.loads(parts[1])
        
        trajectory = data.get('trajectory', [])
        lidar_points = data.get('lidar_points', [])
        
        trajectory_x = [p['x'] for p in trajectory]
        trajectory_y = [p['y'] for p in trajectory]
        
        lidar_x = [p['x'] for p in lidar_points]
        lidar_y = [p['y'] for p in lidar_points]
        
        if trajectory_x and trajectory_y:
            trajectory_line.set_data(trajectory_x, trajectory_y)
        
        if lidar_x and lidar_y:
            lidar_scatter.set_offsets(list(zip(lidar_x, lidar_y)))
        
        if trajectory_x or lidar_x:
            all_x = trajectory_x + lidar_x
            all_y = trajectory_y + lidar_y
            if all_x and all_y:
                margin = 0.5
                ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
                ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        
        print(f"[Visualizer] Updated: {len(trajectory_x)} trajectory points, {len(lidar_x)} lidar points")
        
    except zmq.Again:
        pass
    except Exception as e:
        print(f"[Visualizer] Error: {e}")
    
    return trajectory_line, lidar_scatter

ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=True, cache_frame_data=False)

plt.show()

socket.close()
context.term()
print("[Visualizer] Closed")
