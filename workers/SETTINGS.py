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
# LiDAR Worker Settings
# ============================================
LIDAR_GRID_SIZE = 65
LIDAR_GRID_RESOLUTION = 0.1  # meters per cell
LIDAR_MAP_SIZE = 5.0         # meters (5 x 5)

DIST_UNKNOWN_MM = 65535
DIST_MIN_MM = 20
DIST_MAX_MM = 1400

PLOT_INTERVAL = 0.2  # Update plot max 5 times per second

INFLATION_RADIUS_CELLS = 1

# ============================================
# Path Planning Settings
# ============================================
PATH_PLANNING_ALTITUDE = 10.0   # meters (fixed altitude)
PATH_PLANNING_MAP_SIZE = 40.0   # meters (40 x 40)
PATH_PLANNING_RESOLUTION = 0.2  # meters per cell
PATH_PLANNING_GRID_SIZE = int(PATH_PLANNING_MAP_SIZE / PATH_PLANNING_RESOLUTION)

# Reference GPS coordinates
LAT0 = 37.4275
LON0 = -122.1697

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
