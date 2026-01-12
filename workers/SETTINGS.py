# ============================================
# NIDAR Workers Configuration Settings
# ============================================

# ============================================
# Common Constants
# ============================================
EARTH_RADIUS = 6378137.0  # meters

# ============================================
# Worker IDs
# ============================================
WORKER_ID_PLACEHOLDER = "placeholder_worker"
WORKER_ID_CAMERA = "camera_worker"
WORKER_ID_LIDAR = "lidar_worker"
WORKER_ID_MISSION_MANAGER = "mission_manager"
WORKER_ID_PATH_PLANNER = "path_planner_worker"

# ============================================
# Camera Worker Settings
# ============================================
CAMERA_MODE = "scout"  # "scout" or "sprayer"
CAMERA_ALTITUDE = 5.0  # meters
CAMERA_HFOV_DEG = 78   # degrees
CAMERA_VFOV_DEG = 64   # degrees

REDIS_GEO_KEY = "diseased_crops"
REDIS_EVENT_CHANNEL = "event:crop_detected"
DUPLICATE_RADIUS_METERS = 0.5

# ============================================
# Path Planning Settings
# ============================================
PATH_PLANNING_ALTITUDE = 10.0   # meters (fixed altitude)
PATH_PLANNING_MAP_SIZE = 200.0  # meters (200 x 200) - covers farm polygon
PATH_PLANNING_RESOLUTION = 0.5  # meters per cell
PATH_PLANNING_GRID_SIZE = int(PATH_PLANNING_MAP_SIZE / PATH_PLANNING_RESOLUTION)

# ============================================
# LiDAR Worker Settings
# ============================================
# LiDAR uses the same grid as path planner for consistency
LIDAR_GRID_SIZE = PATH_PLANNING_GRID_SIZE
LIDAR_GRID_RESOLUTION = PATH_PLANNING_RESOLUTION
LIDAR_MAP_SIZE = PATH_PLANNING_MAP_SIZE

DIST_UNKNOWN_MM = 65535
DIST_MIN_MM = 20
DIST_MAX_MM = 1400

PLOT_INTERVAL = 0.2  # Update plot max 5 times per second

INFLATION_RADIUS_CELLS = 1

# Reference GPS coordinates (center of farm polygon from NIDAR.kml)
# Polygon bounds: lat 29.9494-29.9508, lon 76.8159-76.8167
LAT0 = 29.95010  # center latitude of farm
LON0 = 76.81630  # center longitude of farm

MARGIN_DISTANCE_M = 2  # meters

# ============================================
# MAVLink Manager Settings
# ============================================
MAVLINK_MAX_PER_TICK = 50       # fairness cap
MAVLINK_MAX_LATENCY_S = 0.5     # drop stale data

# ============================================
# Mission Manager Settings
# ============================================
DRONES = ("scout", "sprayer")
STATE_REDIS_KEY = "mission:state"
STATE_REDIS_DRONE_KEY = "mission:state:{}"
STATE_PUBLISH_CHANNEL = "mission:state_update"
DEFAULT_ALTITUDE = 5.0  # meters

# Sprayer workflow settings
SPRAYER_CRUISE_ALT = 5.0        # meters - altitude for navigation
SPRAYER_SPRAY_ALT = 2.0         # meters - altitude for spraying (must be > 1m to avoid ground effect)
SPRAYER_WAYPOINT_RADIUS = 1.0   # meters - radius to consider waypoint reached
SPRAYER_CROP_RADIUS = 0.5       # meters - radius to consider crop reached
SPRAYER_HOVER_TIME = 1.0        # seconds - hover time for center correction
SPRAYER_SPRAY_DURATION = 20.0    # seconds - sprayer mechanism trigger duration (mock)

# ============================================
# Occupancy Grid Cell Values
# ============================================
GRID_FREE = 0           # Explored and free to traverse
GRID_OBSTACLE = 1       # Explored and blocked (obstacle)
GRID_UNEXPLORED = 2     # Not yet explored by scout

# ============================================
# Farm Polygon Coordinates (from NIDAR.kml)
# ============================================
FARM_POLYGON = [
    {"lon": 76.81589155096442, "lat": 29.94965311567124},
    {"lon": 76.8162492257966,  "lat": 29.94939808518254},
    {"lon": 76.81664355584154, "lat": 29.94963310749574},
    {"lon": 76.81673785885653, "lat": 29.95054251583795},
    {"lon": 76.81637533963627, "lat": 29.95078200571026},
    {"lon": 76.81601151843628, "lat": 29.95060813092637},
]

def point_in_polygon(lat: float, lon: float, polygon: list = None) -> bool:
    """
    Ray-casting algorithm to check if a point is inside a polygon.
    Uses FARM_POLYGON by default.
    """
    if polygon is None:
        polygon = FARM_POLYGON
    
    n = len(polygon)
    inside = False
    
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]["lon"], polygon[i]["lat"]
        xj, yj = polygon[j]["lon"], polygon[j]["lat"]
        
        if ((yi > lat) != (yj > lat)) and (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    
    return inside
