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
grid_cells_x = []
grid_cells_y = []
grid_cells_prob = []

fig, ax = plt.subplots(figsize=(12, 10))
ax.set_aspect('equal', 'box')
ax.grid(True, alpha=0.3)
ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_title('Occupancy Grid Map + Robot Trajectory')

trajectory_line, = ax.plot([], [], 'r-', linewidth=2, label='Robot Path', zorder=3)
grid_scatter = ax.scatter([], [], c=[], cmap='gray_r', s=10, alpha=0.8, label='Occupancy Grid', 
                          vmin=0.0, vmax=1.0, edgecolors='none', zorder=2)

ax.legend()

def update_plot(frame):
    global trajectory_x, trajectory_y, grid_cells_x, grid_cells_y, grid_cells_prob
    
    socket.setsockopt(zmq.RCVTIMEO, 100)
    
    try:
        full_message = socket.recv_string()
        message_parts = full_message.split(' ', 1)
        
        if len(message_parts) != 2:
            return trajectory_line, grid_scatter
        
        data = json.loads(message_parts[1])
        
        trajectory = data.get('trajectory', [])
        grid_cells = data.get('grid_cells', [])
        
        trajectory_x = [p['x'] for p in trajectory]
        trajectory_y = [p['y'] for p in trajectory]
        
        grid_cells_x = [c['x'] for c in grid_cells]
        grid_cells_y = [c['y'] for c in grid_cells]
        grid_cells_prob = [c['p'] for c in grid_cells]
        
        if trajectory_x and trajectory_y:
            trajectory_line.set_data(trajectory_x, trajectory_y)
        
        if grid_cells_x and grid_cells_y and grid_cells_prob:
            grid_scatter.set_offsets(list(zip(grid_cells_x, grid_cells_y)))
            grid_scatter.set_array(np.array(grid_cells_prob))
        
        if trajectory_x or grid_cells_x:
            all_x = trajectory_x + grid_cells_x
            all_y = trajectory_y + grid_cells_y
            if all_x and all_y:
                margin = 0.5
                ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
                ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        
        print(f"[Visualizer] Updated: {len(trajectory_x)} trajectory points, {len(grid_cells_x)} grid cells")
        
    except zmq.Again:
        pass
    except Exception as e:
        print(f"[Visualizer] Error: {e}")
    
    return trajectory_line, grid_scatter

ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=True, cache_frame_data=False)

plt.show()

socket.close()
context.term()
print("[Visualizer] Closed")
