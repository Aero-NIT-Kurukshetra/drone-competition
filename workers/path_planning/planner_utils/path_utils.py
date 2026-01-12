def simplify_path(path):
    """
    Removes collinear points from a path.
    path: list of (x, y) tuples in map coordinates
    """
    if len(path) < 3:
        return path

    simplified = [path[0]]

    for i in range(1, len(path) - 1):
        x0, y0 = simplified[-1]
        x1, y1 = path[i]
        x2, y2 = path[i + 1]

        # Direction vectors
        dx1, dy1 = x1 - x0, y1 - y0
        dx2, dy2 = x2 - x1, y2 - y1

        # If direction changes, keep point
        if dx1 * dy2 != dy1 * dx2:
            simplified.append((x1, y1))

    simplified.append(path[-1])
    return simplified


def has_line_of_sight(p1, p2, inflated_grid, map_size, resolution):
    """
    Checks if straight line between p1 and p2 is obstacle-free
    p1, p2: (x, y) in map coordinates
    """
    x1, y1 = p1
    x2, y2 = p2

    dist = ((x2 - x1)**2 + (y2 - y1)**2)**0.5
    steps = int(dist / resolution)

    for i in range(steps + 1):
        x = x1 + (x2 - x1) * i / steps
        y = y1 + (y2 - y1) * i / steps

        gx = int((x + map_size/2) / resolution)
        gy = int((y + map_size/2) / resolution)

        if inflated_grid[gx, gy] == 1:
            return False

    return True

def smooth_path_los(path, inflated_grid, map_size, resolution):
    """
    Smooths path using line-of-sight pruning
    """
    if len(path) <= 2:
        return path

    smoothed = [path[0]]
    i = 0

    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if has_line_of_sight(path[i], path[j],
                                 inflated_grid, map_size, resolution):
                break
            j -= 1
        smoothed.append(path[j])
        i = j

    return smoothed
