import numpy as np
from _map import obstacles_1, obstacles_p

def create_map_file(obstacles, filename):
    # Scale and shift coordinates
    scaled_obstacles = np.round((obstacles[:, :2] + 67.5) / 3).astype(int) 
    # scaled_obstacles = np.round((obstacles[:, :2] + 28.5) / 3).astype(int)
    
    # Determine grid size
    max_x = max(scaled_obstacles[:, 0]) + 1
    max_y = max(scaled_obstacles[:, 1]) + 1
    grid_size = max(46,46)
    
    # Create an empty grid
    grid = np.full((grid_size, grid_size), '.') # for test
    # grid = np.full((grid_size, grid_size), '0') # for real
    
    # Place obstacles
    for obs in scaled_obstacles:
        if 0 <= obs[1] < grid_size and 0 <= obs[0] < grid_size:
            grid[obs[1], obs[0]] = '@' # for test
            # grid[obs[1], obs[0]] = '1' # for real
    
    # Write to file
    with open(filename, 'w') as f:
        f.write(f"{grid_size} {grid_size}\n")
        for row in grid:
            f.write(''.join(row) + '\n')

# Your obstacle data
obstacles = np.array([
    # ... (your obstacle data here)
])

create_map_file(obstacles_p, 'map_pen_real_1.txt')