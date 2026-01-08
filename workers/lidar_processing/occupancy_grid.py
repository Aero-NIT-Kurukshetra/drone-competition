import numpy as np

# Map parameters
MAP_SIZE = 40.0        # meters (40 x 40)
RESOLUTION = 0.2       # meters per cell
GRID_SIZE = int(MAP_SIZE / RESOLUTION)

def create_occupancy_grid():
    """
    Creates an empty occupancy grid

    
    """
    """
    0 = unknown
    1 = occupied
    2 = free
    """
    grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)
    return grid




# Map coordinates to grid indices
def map_to_grid(x_m, y_m):
    """
    Converts map-frame coordinates (meters)
    to grid indices
    """
    gx = int((x_m + MAP_SIZE / 2) / RESOLUTION)
    gy = int((y_m + MAP_SIZE / 2) / RESOLUTION)

    if 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE:
        return gx, gy
    else:
        return None


def raytrace_free_space(x0, y0, x1, y1, grid):
    """
    Marks free cells along a ray from (x0,y0) to (x1,y1)
    """
    steps = int(max(abs(x1 - x0), abs(y1 - y0)) / RESOLUTION)
    if steps <= 0:
        return

    for i in range(steps):
        x = x0 + (x1 - x0) * i / steps
        y = y0 + (y1 - y0) * i / steps

        idx = map_to_grid(x, y)
        if idx is not None:
            gx, gy = idx
            # grid[gx, gy] = 2  # mark as free
            if grid[gx, gy] == 0 :  # don't overwrite occupied
                grid[gx, gy] = 2



# Update occupancy grid from LiDAR points
def update_grid_from_scan(points_map,grid,drone_pose):
    """
    Updates occupancy grid using MAP-frame points
    Uses ray tracing to mark free space and obstacles
    
    """
    x_d, y_d, yaw = drone_pose
    for (x_m, y_m) in points_map:
        # print(f"Point: ({x_m:.2f}, {y_m:.2f}), Distance: {d} mm")
        # if d >= 17000:
        #     continue  # skip max range points

        raytrace_free_space(x_d, y_d, x_m, y_m, grid)
        idx = map_to_grid(x_m, y_m)
        if idx is not None:
            gx, gy = idx
            grid[gx, gy] = 1  # mark as occupied






# Decide inflation radius (engineering choice)

# Let’s choose realistic values:

# Drone radius ≈ 0.4 m

# GPS / estimation error ≈ 0.3 m

# Safety margin ≈ 0.2 m

# Total inflation radius:
# 0.4 + 0.3 + 0.2 = 0.9 m


# If your grid resolution is:

# RESOLUTION = 0.2 m/cell

# Inflation radius in grid cells:
# 0.9 / 0.2 ≈ 5 cells


# So we will inflate obstacles by 5 cells in all directions.
import numpy as np

def inflate_obstacles(grid, inflation_radius_cells):
    """
    Inflates obstacle cells by given radius (in grid cells)
    """
    inflated_grid = grid.copy()
    obstacle_cells = np.argwhere(grid == 1)

    for gx, gy in obstacle_cells:
        for dx in range(-inflation_radius_cells, inflation_radius_cells + 1):
            for dy in range(-inflation_radius_cells, inflation_radius_cells + 1):

                nx = gx + dx
                ny = gy + dy

                if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
                    inflated_grid[nx, ny] = 1

    return inflated_grid
# Inflate obstacles