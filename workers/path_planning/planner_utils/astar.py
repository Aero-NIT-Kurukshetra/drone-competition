import heapq
import math

def heuristic(a, b):
    # Euclidean distance
    return math.hypot(a[0] - b[0], a[1] - b[1])

def astar(grid, start, goal):
    """
    grid: inflated occupancy grid
    start, goal: (gx, gy)
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

            if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                continue

            # ❌ blocked cell
            if grid[neighbor[0], neighbor[1]] == 1:
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
