from collections import deque

def nearest_free_cell(grid, start):
    """
    Finds nearest free cell (BFS search)
    """
    rows, cols = grid.shape
    visited = set()
    queue = deque([start])

    while queue:
        cx, cy = queue.popleft()

        if (cx, cy) in visited:
            continue
        visited.add((cx, cy))

        # ✅ Free cell found
        if grid[cx, cy] == 2:
            return (cx, cy)

        # Explore neighbors
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),
                       (-1,-1),(-1,1),(1,-1),(1,1)]:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < rows and 0 <= ny < cols:
                queue.append((nx, ny))

    return None  # No free cell found



