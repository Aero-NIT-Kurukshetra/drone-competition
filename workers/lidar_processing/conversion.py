import math

EARTH_RADIUS = 6378137  # meters

def gps_to_local(lat, lon, lat0, lon0):
    """
    Converts GPS to local ENU coordinates (meters)
    """
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)

    x = dlon * math.cos(math.radians(lat0)) * EARTH_RADIUS
    y = dlat * EARTH_RADIUS

    return x, y






def polar_to_cartesian(angle_deg, distance_mm):
    """
    Converts polar LiDAR data to Cartesian (LiDAR frame)
    Returns (x, y) in meters
    """
    r = distance_mm / 1000.0      # mm → meters
    theta = math.radians(angle_deg)

    x = r * math.cos(theta)
    y = r * math.sin(theta)

    return x, y



def lidar_to_map(x_l, y_l, x_d, y_d, yaw_deg):
    """
    Transforms LiDAR-frame point to MAP frame given drone pose
    Returns (x_m, y_m) in meters
    """
    yaw = math.radians(yaw_deg)
    #x_m and y_m are the coordinates of the point in the map frame
    x_m = x_d + x_l * math.cos(yaw) - y_l * math.sin(yaw)
    y_m = y_d + x_l * math.sin(yaw) + y_l * math.cos(yaw)


    return x_m, y_m

import math

EARTH_RADIUS = 6378137.0  # meters

def map_to_gps(x, y, lat0, lon0):
    """
    Convert local map (x,y) in meters to GPS (lat,lon)
    """
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)

    dlat = y / EARTH_RADIUS
    dlon = x / (EARTH_RADIUS * math.cos(lat0_rad))

    lat = lat0_rad + dlat
    lon = lon0_rad + dlon

    return math.degrees(lat), math.degrees(lon)


def grid_to_map(gx, gy, map_size, resolution):
    """
    Converts grid indices to map coordinates (meters)
    """
    x = gx * resolution - map_size / 2
    y = gy * resolution - map_size / 2
    return x, y
