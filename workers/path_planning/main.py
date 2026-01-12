import asyncio
import logging
import time
import math
import xml.etree.ElementTree as ET
import numpy as np

from shapely.geometry import Polygon, LineString, MultiLineString
from shapely.affinity import rotate, translate
from pyproj import Transformer
import json

from common.redis_client import RedisClient

from workers.path_planning.planner_utils.astar import astar
from workers.path_planning.planner_utils.conversion import map_to_grid, grid_to_map,map_to_gps,gps_to_local
from workers.path_planning.planner_utils.path_utils import simplify_path,has_line_of_sight,smooth_path_los
from workers.SETTINGS import (
    WORKER_ID_PATH_PLANNER,
    PATH_PLANNING_ALTITUDE,
    PATH_PLANNING_MAP_SIZE,
    PATH_PLANNING_RESOLUTION,
    PATH_PLANNING_GRID_SIZE,
    LAT0,
    LON0,
    MARGIN_DISTANCE_M,
    GRID_FREE,
    GRID_OBSTACLE,
    GRID_UNEXPLORED,
)

ALTITUDE = PATH_PLANNING_ALTITUDE
MAP_SIZE = PATH_PLANNING_MAP_SIZE
RESOLUTION = PATH_PLANNING_RESOLUTION
GRID_SIZE = PATH_PLANNING_GRID_SIZE


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("path_planner_worker")

loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)
shutdown_event = asyncio.Event()

WORKER_ID = WORKER_ID_PATH_PLANNER
redis = RedisClient(loop=loop, worker_id=WORKER_ID)

def read_kml_polygon_from_xml(kml_xml: str) -> Polygon:
    root = ET.fromstring(kml_xml)
    ns = {"kml": "http://www.opengis.net/kml/2.2"}

    coord_elem = root.find(".//kml:Polygon//kml:coordinates", ns)
    if coord_elem is None:
        raise ValueError("No Polygon found in KML XML")

    coords = []
    for token in coord_elem.text.strip().split():
        lon, lat, *_ = map(float, token.split(","))
        coords.append((lon, lat))

    if len(coords) < 3:
        raise ValueError("Polygon must have at least 3 points")

    if coords[0] != coords[-1]:
        coords.append(coords[0])

    return Polygon(coords)

def generate_lawnmower(poly: Polygon, spacing_m: float, angle_deg: float):
    to_m = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
    to_ll = Transformer.from_crs("EPSG:3857", "EPSG:4326", always_xy=True)

    poly_m = Polygon([to_m.transform(x, y) for x, y in poly.exterior.coords])

    # safety margin
    poly_m = poly_m.buffer(-4)
    if poly_m.is_empty:
        raise ValueError("Polygon collapsed after safety buffer")

    minx, miny, maxx, maxy = poly_m.bounds
    cx, cy = (minx + maxx) / 2, (miny + maxy) / 2
    diag = math.hypot(maxx - minx, maxy - miny)

    # Generate parallel sweep lines
    lines = []
    y = -diag
    while y <= diag:
        lines.append(LineString([(-diag, y), (diag, y)]))
        y += spacing_m

    # Rotate and translate to polygon center
    lines = [rotate(l, angle_deg, origin=(0, 0)) for l in lines]
    lines = [translate(l, xoff=cx, yoff=cy) for l in lines]

    # Clip to polygon
    clipped = []
    for line in lines:
        inter = poly_m.intersection(line)
        if inter.is_empty:
            continue
        if isinstance(inter, LineString):
            clipped.append(inter)
        elif isinstance(inter, MultiLineString):
            clipped.extend(inter.geoms)

    return clipped, to_ll

def generate_waypoints(lines, transformer, angle_deg):
    def order_key(line):
        c = line.centroid
        theta = math.radians(angle_deg + 90)
        return c.x * math.cos(theta) + c.y * math.sin(theta)

    lines = [l for l in lines if not l.is_empty]
    lines.sort(key=order_key)

    waypoints = []

    for i, line in enumerate(lines):
        pts = list(line.coords)

        # zig-zag ordering
        if i % 2 == 1:
            pts.reverse()

        for x, y in pts:
            lon, lat = transformer.transform(x, y)
            waypoints.append({
                "lat": round(lat, 9),
                "lon": round(lon, 9),
                "alt_m": 5.0
            })

    return waypoints

@redis.listen("mission_manager:scout_planning_request")
async def handle_pathplanner_event(data: dict):
    """
    Expected payload:
    {
        "kml_xml": "<kml>...</kml>",
        "spacing": 5,
        "angle": 60
    }
    """
    logger.info(f"[{WORKER_ID}] Task received")

    try:
        polygon = read_kml_polygon_from_xml(data["kml_xml"])

        lines, transformer = generate_lawnmower(
            polygon,
            spacing_m=float(data["spacing"]),
            angle_deg=float(data["angle"])
        )

        waypoints = generate_waypoints(
            lines,
            transformer,
            angle_deg=float(data["angle"])
        )

        # Print waypoints to terminal
        logger.info("Generated waypoints:")
        for i, wp in enumerate(waypoints, 1):
            logging.info(f"{i:03d}: {wp}")

        await redis.client.set(
            "path_planner:scout_waypoints",
            json.dumps({
                "worker_id": WORKER_ID,
                "waypoints": waypoints,
                "timestamp": time.time()
            })
        )
        await redis.client.set("path_planner:current_scout_waypoint_index", "0")
        
        # Initialize occupancy grid as fully unexplored (only if not already set)
        existing_grid = await redis.binary_client.get("occupancy_grid")
        if existing_grid is None:
            await initialize_unexplored_grid()
        else:
            logger.info(f"[{WORKER_ID}] Occupancy grid already exists, skipping initialization")

    except Exception as e:
        logger.exception("Path planning failed")
        await redis.publish(
            "event:pathplanner_out",
            {
                "worker_id": WORKER_ID,
                "status": "error",
                "error": str(e),
                "timestamp": time.time()
            }
        )


# ============================================
# EXPLORATION LOGIC
# ============================================
# Exploration radius in meters - area marked explored around scout
SCOUT_EXPLORATION_RADIUS_M = 10.0
SCOUT_EXPLORATION_RADIUS_CELLS = int(SCOUT_EXPLORATION_RADIUS_M / RESOLUTION)


async def initialize_unexplored_grid():
    """Initialize occupancy grid as fully unexplored (value=2)."""
    grid = np.full((GRID_SIZE, GRID_SIZE), GRID_UNEXPLORED, dtype=np.uint8)
    await redis.binary_client.set("occupancy_grid", grid.tobytes())
    logger.info(f"[{WORKER_ID}] Initialized {GRID_SIZE}x{GRID_SIZE} unexplored grid")


async def mark_cells_explored_by_radius_m(center_lat: float, center_lon: float, radius_m: float):
    """Mark cells around a GPS position as explored using radius in meters."""
    radius_cells = int(radius_m / RESOLUTION)
    await mark_cells_explored(center_lat, center_lon, radius_cells)


async def mark_cells_explored(center_lat: float, center_lon: float, radius_cells: int):
    """
    Mark cells around a GPS position as explored (free=0).
    This is called when the scout moves to reveal the map.
    """
    # Get current grid
    grid_raw = await redis.binary_client.get("occupancy_grid")
    if not grid_raw:
        logger.warning(f"[{WORKER_ID}] No occupancy grid found, initializing...")
        await initialize_unexplored_grid()
        grid_raw = await redis.binary_client.get("occupancy_grid")
    
    grid = np.frombuffer(grid_raw, dtype=np.uint8).reshape((GRID_SIZE, GRID_SIZE)).copy()
    
    # Convert GPS to grid coordinates
    x, y = gps_to_local(center_lat, center_lon, LAT0, LON0)
    center_cell = map_to_grid(x, y)
    
    if center_cell is None:
        return  # Out of bounds
    
    cx, cy = center_cell
    cells_marked = 0
    
    # Mark circular area as explored
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            # Check if within circular radius
            if dx*dx + dy*dy <= radius_cells*radius_cells:
                gx, gy = cx + dx, cy + dy
                
                if 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE:
                    # Only mark unexplored cells as free (preserve obstacles)
                    if grid[gy, gx] == GRID_UNEXPLORED:
                        grid[gy, gx] = GRID_FREE
                        cells_marked += 1
    
    if cells_marked > 0:
        await redis.binary_client.set("occupancy_grid", grid.tobytes())
        logger.debug(f"[{WORKER_ID}] Marked {cells_marked} cells explored around ({center_lat:.6f}, {center_lon:.6f})")


@redis.listen("mission_manager:drone_pose_update")
async def handle_scout_exploration(data: dict):
    """
    When scout moves, mark surrounding cells as explored.
    Only processes scout drone updates.
    """
    drone_id = data.get("drone_id")
    
    if drone_id != "scout":
        return  # Only scout explores
    
    lat = data.get("lat")
    lon = data.get("lon")
    
    if lat is None or lon is None:
        return
    
    await mark_cells_explored(lat, lon, SCOUT_EXPLORATION_RADIUS_CELLS)


@redis.listen("path_planning:mark_explored")
async def handle_mark_explored(data: dict):
    """
    Explicitly mark an area as explored.
    Called when crop is detected to ensure the area is marked explored
    before sprayer planning starts.
    
    Payload:
    {
        "lat": float,
        "lon": float,
        "radius_m": float (optional, default 15.0)
    }
    """
    lat = data.get("lat")
    lon = data.get("lon")
    radius_m = data.get("radius_m", 15.0)
    
    if lat is None or lon is None:
        return
    
    logger.info(f"[{WORKER_ID}] Marking {radius_m}m radius explored at ({lat:.6f}, {lon:.6f})")
    await mark_cells_explored_by_radius_m(lat, lon, radius_m)


@redis.listen("mission_manager:sprayer_plan_request")
async def handle_sprayer_planning_request(data: dict):
    """
    Sprayer planning per workflow:
    1. Increment current_crop_target_index + 1
    2. Fetch crop from crop_locations[] using index
    3. Compute path[] using A*
    4. Save to path_planner:sprayer_waypoints[]
    5. Set current_sprayer_waypoint_index = 0
    6. Publish planned_waypoint with first waypoint
    """

    logger.info(f"[{WORKER_ID}] Sprayer planning request received")

    drone_pose = data.get("drone_pose", {})
    
    if not drone_pose or drone_pose.get("lat") is None or drone_pose.get("lon") is None:
        logger.error(f"[{WORKER_ID}] Invalid drone pose: {drone_pose}")
        await redis.publish(
            "path_planning:planned_waypoint",
            {"drone_id": "sprayer", "waypoint": None, "error": "invalid_pose"}
        )
        return
    
    # Step 1: Get current index and increment it
    current_index_raw = await redis.client.get("path_planner:current_crop_target_index")
    current_index = int(current_index_raw) if current_index_raw else -1
    target_index = current_index + 1
    
    # Step 2: Fetch crop_locations array
    crop_locations_raw = await redis.client.get("crop_locations")
    
    if not crop_locations_raw:
        logger.warning(f"[{WORKER_ID}] No crop locations available")
        await redis.publish(
            "path_planning:planned_waypoint",
            {"drone_id": "sprayer", "waypoint": None}
        )
        return
    
    crop_locations = json.loads(crop_locations_raw)
    
    # Check bounds
    if target_index >= len(crop_locations):
        logger.info(f"[{WORKER_ID}] All crops processed ({len(crop_locations)} total)")
        await redis.publish(
            "path_planning:planned_waypoint",
            {"drone_id": "sprayer", "waypoint": None}
        )
        return
    
    # Update index NOW (we're starting to process this crop)
    await redis.client.set("path_planner:current_crop_target_index", str(target_index))
    
    target_crop = crop_locations[target_index]
    
    logger.info(
        f"[{WORKER_ID}] Planning path to crop {target_index + 1}/{len(crop_locations)} "
        f"at ({target_crop['lat']:.6f}, {target_crop['lon']:.6f})"
    )
    
    # Step 3: Compute path using A*
    occupancy_grid_raw = await redis.binary_client.get("occupancy_grid")
    
    if not occupancy_grid_raw:
        logger.warning(f"[{WORKER_ID}] No occupancy grid available, using direct path")
        # Fallback: direct path to target
        sprayer_waypoints = [{
            "lat": target_crop["lat"],
            "lon": target_crop["lon"],
            "alt_m": target_crop.get("alt_m", target_crop.get("alt", 5.0))
        }]
    else:
        # Decode bytes back to numpy array (make a copy since frombuffer is read-only)
        inflated_grid = np.frombuffer(occupancy_grid_raw, dtype=np.uint8).reshape(
            (GRID_SIZE, GRID_SIZE)
        ).copy()
        
        x_start, y_start = gps_to_local(drone_pose.get("lat", LAT0), drone_pose.get("lon", LON0), LAT0, LON0)
        x_goal, y_goal = gps_to_local(target_crop['lat'], target_crop['lon'], LAT0, LON0)
        
        start_cell = map_to_grid(x_start, y_start)
        goal_cell  = map_to_grid(x_goal, y_goal)

        # Handle out-of-bounds cells
        if start_cell is None or goal_cell is None:
            logger.warning(f"[{WORKER_ID}] Start or goal out of grid bounds")
            await redis.publish(
                "event:no_safe_path",
                {"drone_id": "sprayer", "reason": "out_of_bounds"}
            )
            return
        
        start_gx, start_gy = start_cell
        goal_gx, goal_gy = goal_cell
        
        # Ensure sprayer's start position is marked as explored/free
        if inflated_grid[start_gy, start_gx] == GRID_UNEXPLORED:
            logger.info(f"[{WORKER_ID}] Marking sprayer start position as explored")
            inflated_grid[start_gy, start_gx] = GRID_FREE
            # Also mark a small area around sprayer as explored (5m radius)
            sprayer_radius = int(5.0 / RESOLUTION)
            for dx in range(-sprayer_radius, sprayer_radius + 1):
                for dy in range(-sprayer_radius, sprayer_radius + 1):
                    if dx*dx + dy*dy <= sprayer_radius*sprayer_radius:
                        gx, gy = start_gx + dx, start_gy + dy
                        if 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE:
                            if inflated_grid[gy, gx] == GRID_UNEXPLORED:
                                inflated_grid[gy, gx] = GRID_FREE
        
        # Check if goal is in unexplored area
        if inflated_grid[goal_gy, goal_gx] == GRID_UNEXPLORED:
            logger.info(f"[{WORKER_ID}] Crop {target_index + 1} is in unexplored area, waiting for scout...")
            # Revert index since we couldn't process this crop yet
            await redis.client.set("path_planner:current_crop_target_index", str(target_index - 1))
            await redis.publish(
                "event:sprayer_waiting",
                {
                    "drone_id": "sprayer", 
                    "reason": "crop_unexplored",
                    "crop_index": target_index,
                    "crop_lat": target_crop["lat"],
                    "crop_lon": target_crop["lon"]
                }
            )
            return
        
        logger.info(f"[{WORKER_ID}] Running A* from grid({start_gx},{start_gy}) to grid({goal_gx},{goal_gy})")
        path = astar(inflated_grid, start_cell, goal_cell)

        # Handle no path found
        if path is None or len(path) == 0:
            logger.warning(f"[{WORKER_ID}] A* found no path to crop")
            # Revert index since we couldn't process this crop yet
            await redis.client.set("path_planner:current_crop_target_index", str(target_index - 1))
            await redis.publish(
                "event:sprayer_waiting",
                {
                    "drone_id": "sprayer", 
                    "reason": "path_blocked",
                    "crop_index": target_index
                }
            )
            return
        
        map_path = []
        for gx, gy in path:
            x, y = grid_to_map(gx, gy, MAP_SIZE, RESOLUTION)
            map_path.append((x, y))
        
        simplified_path = simplify_path(map_path)
        smoothed_path = smooth_path_los(
            simplified_path,
            inflated_grid,
            MAP_SIZE,
            RESOLUTION
        )

        # Fallback if smoothed path is empty
        if len(smoothed_path) == 0:
            logger.warning(f"[{WORKER_ID}] Smoothed path empty")
            await redis.publish(
                "event:no_safe_path",
                {"drone_id": "sprayer", "reason": "empty_path"}
            )
            return

        sprayer_waypoints = []
        for x, y in smoothed_path:
            wp_lat, wp_lon = map_to_gps(x, y, LAT0, LON0)
            sprayer_waypoints.append({
                "lat": wp_lat,
                "lon": wp_lon,
                "alt_m": target_crop.get("alt_m", target_crop.get("alt", 5.0))
            })
    
    # Store waypoints in Redis
    await redis.client.set(
        "path_planner:sprayer_waypoints",
        json.dumps({
            "worker_id": WORKER_ID,
            "waypoints": sprayer_waypoints,
            "target_crop_index": target_index,
            "timestamp": time.time()
        })
    )
    
    # Reset sprayer waypoint index to 0
    await redis.client.set("path_planner:current_sprayer_waypoint_index", "0")
    
    logger.info(f"[{WORKER_ID}] Sprayer waypoints set: {len(sprayer_waypoints)} points")
    
    # Publish first waypoint to mission manager
    await redis.publish(
        "path_planning:planned_waypoint",
        {"drone_id": "sprayer", "waypoint": sprayer_waypoints[0]}
    )
    

def haversine_distance(lat1, lon1, lat2, lon2):
    # Earth radius in meters
    R = 6371000  

    # Convert degrees to radians
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    # Haversine formula
    a = math.sin(dphi / 2)**2 + \
        math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2)**2

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c

@redis.listen("mission_manager:request_next_waypoint")
async def handle_request_next_waypoint(data):
    drone_id = data["drone_id"]
    lat = data.get("lat", None)
    lon = data.get("lon", None)
    
    logger.info(f"[{WORKER_ID}] Planning request for {drone_id}")

    if drone_id == "scout":
        await _handle_scout_waypoint_request(lat, lon)
    elif drone_id == "sprayer":
        await _handle_sprayer_waypoint_request(lat, lon)
    else:
        logger.error(f"[{WORKER_ID}] Unknown drone_id: {drone_id}")


async def _handle_scout_waypoint_request(lat, lon):
    """Handle next waypoint request for scout drone."""
    current_scout_waypoints = await redis.client.get("path_planner:scout_waypoints")
    if current_scout_waypoints is None:
        logger.error(f"[{WORKER_ID}] No scout waypoints available")
        await redis.publish(
            "path_planning:planned_waypoint",
            {"drone_id": "scout", "waypoint": None}
        )
        return
    
    current_scout_waypoints = json.loads(current_scout_waypoints)
    current_scout_waypoints = current_scout_waypoints["waypoints"]

    current_waypoint_raw = await redis.client.get("path_planner:current_scout_waypoint_index")
    current_waypoint = int(current_waypoint_raw) if current_waypoint_raw else 0

    if current_waypoint >= len(current_scout_waypoints) - 1: 
        logger.info(f"[{WORKER_ID}] All waypoints completed for scout")
        await redis.publish(
            "path_planning:planned_waypoint",
            {"drone_id": "scout", "waypoint": None}
        )
    else:
        curr_wp = current_scout_waypoints[current_waypoint]
        next_wp = current_scout_waypoints[current_waypoint+1]
        logger.info(f"[{WORKER_ID}] Sending waypoint {current_waypoint + 1}/{len(current_scout_waypoints)} to scout: {next_wp}")

        pend_lat = curr_wp["lat"]
        pend_lon = curr_wp["lon"]

        if lat is not None and lon is not None:
            dist = haversine_distance(lat, lon, pend_lat, pend_lon)

            if dist > MARGIN_DISTANCE_M:
                logger.warning(f"[{WORKER_ID}] Scout is {dist:.2f}m away from expected position for waypoint {current_waypoint}. Sending current waypoint.")
                await redis.publish(
                    "path_planning:planned_waypoint",
                    {"drone_id": "scout", "waypoint": curr_wp}
                )
                return

        await redis.publish(
            "path_planning:planned_waypoint",
            {"drone_id": "scout", "waypoint": next_wp}
        )
        await redis.client.publish("path_planner:total_scout_waypoints", str(len(current_scout_waypoints)))
        await redis.client.set("path_planner:current_scout_waypoint_index", str(current_waypoint + 1))


async def _handle_sprayer_waypoint_request(lat, lon):
    """Handle next waypoint request for sprayer drone."""
    sprayer_waypoints_raw = await redis.client.get("path_planner:sprayer_waypoints")
    
    if sprayer_waypoints_raw is None:
        logger.warning(f"[{WORKER_ID}] No sprayer waypoints available")
        await redis.publish(
            "path_planning:planned_waypoint",
            {"drone_id": "sprayer", "waypoint": None}
        )
        return
    
    sprayer_data = json.loads(sprayer_waypoints_raw)
    sprayer_waypoints = sprayer_data["waypoints"]

    current_wp_raw = await redis.client.get("path_planner:current_sprayer_waypoint_index")
    current_waypoint = int(current_wp_raw) if current_wp_raw else 0

    if current_waypoint >= len(sprayer_waypoints):
        logger.info(f"[{WORKER_ID}] All sprayer waypoints completed")
        await redis.publish(
            "path_planning:planned_waypoint",
            {"drone_id": "sprayer", "waypoint": None}
        )
        return

    curr_wp = sprayer_waypoints[current_waypoint]
    
    # Check if this is first waypoint or we need to verify position
    if current_waypoint > 0 and lat is not None and lon is not None:
        prev_wp = sprayer_waypoints[current_waypoint - 1]
        dist = haversine_distance(lat, lon, prev_wp["lat"], prev_wp["lon"])
        
        if dist > MARGIN_DISTANCE_M:
            logger.warning(f"[{WORKER_ID}] Sprayer is {dist:.2f}m away from expected position. Resending waypoint {current_waypoint}.")
            await redis.publish(
                "path_planning:planned_waypoint",
                {"drone_id": "sprayer", "waypoint": curr_wp}
            )
            return

    logger.info(f"[{WORKER_ID}] Sending waypoint {current_waypoint + 1}/{len(sprayer_waypoints)} to sprayer: {curr_wp}")
    
    await redis.publish(
        "path_planning:planned_waypoint",
        {"drone_id": "sprayer", "waypoint": curr_wp}
    )

async def heartbeat_loop():
    while not shutdown_event.is_set():
        try:
            await redis.heartbeat()
        except Exception as e:
            logger.error(f"Heartbeat error: {e}")
        await asyncio.sleep(1)

async def main(loop):
    await redis.connect()

    mode = await redis.get_startup_mode()
    logger.info(f"[{WORKER_ID}] Starting in {mode} mode")

    tasks = [
        loop.create_task(heartbeat_loop())
    ]

    await shutdown_event.wait()

    logger.info(f"[{WORKER_ID}] Shutting down")
    for t in tasks:
        t.cancel()

    await redis.close()

def runner():
    try:
        loop.run_until_complete(main(loop))
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_event.set()
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()

if __name__ == "__main__":
    runner()
