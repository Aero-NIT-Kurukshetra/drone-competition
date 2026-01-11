import asyncio
import logging
import time
import math
import xml.etree.ElementTree as ET

from shapely.geometry import Polygon, LineString, MultiLineString
from shapely.affinity import rotate, translate
from pyproj import Transformer
import json

from common.redis_client import RedisClient

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("path_planner_worker")

loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)
shutdown_event = asyncio.Event()

WORKER_ID = "path_planner_worker"
redis = RedisClient(loop=loop, worker_id=WORKER_ID)

MARGIN_DISTANCE_M = 2  # meters

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

@redis.listen("event:pathplanner_in")
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

@redis.listen("path_planning:planning_request")
async def handle_planning_request(data):
    drone_id = data["drone_id"]
    lat = data.get("lat", None)
    lon = data.get("lon", None)
    
    logger.info(f"[{WORKER_ID}] Planning request for {drone_id}")

    current_scout_waypoints = await redis.client.get("path_planner:scout_waypoints")
    if current_scout_waypoints is None:
        logger.error(f"[{WORKER_ID}] No waypoints available")
        return
    
    current_scout_waypoints = json.loads(current_scout_waypoints)
    current_scout_waypoints = current_scout_waypoints["waypoints"]

    current_waypoint = int(await redis.client.get("path_planner:current_scout_waypoint_index"))

    if current_waypoint >= len(current_scout_waypoints) - 1:
        logger.info(f"[{WORKER_ID}] All waypoints completed for {drone_id}")
        return
    else:
        curr_wp = current_scout_waypoints[current_waypoint]
        next_wp = current_scout_waypoints[current_waypoint+1]
        logger.info(f"[{WORKER_ID}] Sending waypoint {current_waypoint + 1}/{len(current_scout_waypoints)} to {drone_id}: {next_wp}")

        pend_lat = curr_wp["lat"]
        pend_lon = curr_wp["lon"]

        if lat is not None and lon is not None:
            dist = haversine_distance(lat, lon, pend_lat, pend_lon)

            if dist > MARGIN_DISTANCE_M:
                logger.warning(f"[{WORKER_ID}] Drone is {dist:.2f}m away from expected position for waypoint {current_waypoint}. Sending current waypoint.")
                await redis.publish(
                    "event:planned_waypoint",
                    {"drone_id": drone_id, "waypoint": curr_wp}
                )
                return

        await redis.publish(
            "event:planned_waypoint",
            {"drone_id": drone_id, "waypoint": next_wp}
        )

        await redis.client.set("path_planner:current_scout_waypoint_index", str(current_waypoint + 1))

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
