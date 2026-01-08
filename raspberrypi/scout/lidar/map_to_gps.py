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
