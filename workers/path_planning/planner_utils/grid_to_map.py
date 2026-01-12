def grid_to_map(gx, gy, map_size, resolution):
    """
    Converts grid indices to map coordinates (meters)
    """
    x = gx * resolution - map_size / 2
    y = gy * resolution - map_size / 2
    return x, y
