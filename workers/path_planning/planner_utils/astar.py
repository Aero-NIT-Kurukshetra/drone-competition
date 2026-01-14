import heapq
import math

def heuristic(a, b):
    # Euclidean distance
    return math.hypot(a[0] - b[0], a[1] - b[1])

def astar(grid, start, goal):
    """
    A* pathfinding on occupancy grid.
    
    Grid cell values:
        0 = explored, free (traversable)
        1 = explored, obstacle (blocked)
        2 = unexplored (blocked - sprayer must wait for scout)
    
    Args:
        grid: occupancy grid (numpy array) - indexed as grid[row, col] = grid[gy, gx]
        start: (gx, gy) start cell
        goal: (gx, gy) goal cell
    
    Returns:
        List of (gx, gy) tuples representing path, or None if no path found
    """
    rows, cols = grid.shape

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),
                       (-1,-1),(-1,1),(1,-1),(1,1)]:

            neighbor = (current[0] + dx, current[1] + dy)
            ngx, ngy = neighbor

            # Check bounds: rows = GRID_SIZE (y), cols = GRID_SIZE (x)
            if not (0 <= ngx < cols and 0 <= ngy < rows):
                continue

            # Grid is indexed as grid[row, col] = grid[gy, gx]
            cell_value = grid[ngy, ngx]
            if cell_value != 0:  # Only traverse explored free cells (value=0)
                continue

            tentative_g = g_score[current] + heuristic(current, neighbor)

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, neighbor))

    return None  # no path found

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]
