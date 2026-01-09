import asyncio
import logging
import time
import math
import csv
import xml.etree.ElementTree as ET

from shapely.geometry import Polygon, LineString, MultiLineString
from shapely.affinity import rotate, translate
from pyproj import Transformer
import simplekml

from common.redis_client import RedisClient

# --------------------------------------------------
# Logging
# --------------------------------------------------

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("path_planner_worker")

# --------------------------------------------------
# Async / Worker Setup
# --------------------------------------------------

loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)
shutdown_event = asyncio.Event()

WORKER_ID = "path_planner_worker"
redis = RedisClient(loop=loop, worker_id=WORKER_ID)

# --------------------------------------------------
# Path Planning Logic
# --------------------------------------------------

def read_kml_polygon(kml_file: str) -> Polygon:
    tree = ET.parse(kml_file)
    root = tree.getroot()
    ns = {"kml": "http://www.opengis.net/kml/2.2"}

    coord_elem = root.find(".//kml:Polygon//kml:coordinates", ns)
    if coord_elem is None:
        raise ValueError("No Polygon found in input KML")

    coords = []
    for token in coord_elem.text.strip().split():
        lon, lat, *_ = map(float, token.split(","))
        coords.append((lon, lat))

    if coords[0] != coords[-1]:
        coords.append(coords[0])

    return Polygon(coords)


def generate_lawnmower(poly: Polygon, spacing_m: float, angle_deg: float):
    to_m = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
    to_ll = Transformer.from_crs("EPSG:3857", "EPSG:4326", always_xy=True)

    poly_m = Polygon([to_m.transform(x, y) for x, y in poly.exterior.coords])
    poly_m = poly_m.buffer(-4)

    minx, miny, maxx, maxy = poly_m.bounds
    cx, cy = (minx + maxx) / 2, (miny + maxy) / 2
    diag = math.hypot(maxx - minx, maxy - miny)

    lines = []
    y = -diag
    while y <= diag:
        lines.append(LineString([(-diag, y), (diag, y)]))
        y += spacing_m

    lines = [rotate(l, angle_deg, origin=(0, 0)) for l in lines]
    lines = [translate(l, xoff=cx, yoff=cy) for l in lines]

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


def export_kml_and_csv(lines, transformer, kml_out, csv_out, angle_deg):
    kml = simplekml.Kml()

    def order_key(line):
        c = line.centroid
        theta = math.radians(angle_deg + 90)
        return c.x * math.cos(theta) + c.y * math.sin(theta)

    lines = [l for l in lines if not l.is_empty]
    lines.sort(key=order_key)

    final_coords = []
    point_index = 1

    for i, line in enumerate(lines):
        pts = list(line.coords)
        if i % 2 == 1:
            pts.reverse()

        coords_ll = [(*transformer.transform(x, y), 0) for x, y in pts]
        final_coords.extend(coords_ll)

        ls = kml.newlinestring(
            name=f"Line {i+1}",
            coords=coords_ll
        )
        ls.style.linestyle.width = 2
        ls.style.linestyle.color = simplekml.Color.red

        for lon, lat, _ in coords_ll:
            pt = kml.newpoint(
                name=str(point_index),
                coords=[(lon, lat)]
            )
            pt.style.iconstyle.scale = 0
            pt.style.labelstyle.scale = 0.9
            pt.style.labelstyle.color = simplekml.Color.white
            point_index += 1

    kml.save(kml_out)

    with open(csv_out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["lat", "lon", "alt_m"])
        for lon, lat, _ in final_coords:
            writer.writerow([f"{lat:.9f}", f"{lon:.9f}", "100.000"])

    return final_coords


# --------------------------------------------------
# Redis Event Handler
# --------------------------------------------------

@redis.listen("event:pathplanner_in")
async def handle_pathplanner_event(data: dict):
    """
    Expected input payload:
    {
        "kml_path": "input.kml",
        "spacing": 5,
        "angle": 60,
        "output_kml": "output.kml",
        "output_csv": "waypoints.csv"
    }
    """
    logger.info(f"[{WORKER_ID}] Received task: {data}")

    try:
        polygon = read_kml_polygon(data["kml_path"])
        lines, transformer = generate_lawnmower(
            polygon,
            spacing_m=float(data["spacing"]),
            angle_deg=float(data["angle"])
        )

        coords = export_kml_and_csv(
            lines,
            transformer,
            data["output_kml"],
            data["output_csv"],
            angle_deg=float(data["angle"])
        )

        await redis.publish(
            "event:pathplanner_out",
            {
                "worker_id": WORKER_ID,
                "status": "success",
                "waypoint_count": len(coords),
                "output_kml": data["output_kml"],
                "output_csv": data["output_csv"],
                "timestamp": time.time()
            }
        )

        logger.info(f"[{WORKER_ID}] Path planning completed")

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


# --------------------------------------------------
# Heartbeat & Main Loop
# --------------------------------------------------

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
