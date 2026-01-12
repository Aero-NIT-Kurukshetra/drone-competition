import asyncio
import json
import logging
import sys
from pathlib import Path

from common.redis_client import RedisClient

# ---------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
)
logger = logging.getLogger("pathplanner_test_sender")

# ---------------------------------------------------------------------
# Async setup
# ---------------------------------------------------------------------
loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

WORKER_ID = "pathplanner_test_sender"
redis = RedisClient(loop=loop, worker_id=WORKER_ID)

waypoints = {
    "worker_id": "path_planner_worker", 
    "waypoints": [
        {"lat": 29.949619679, "lon": 76.816557621, "alt_m": 5.0}, 
        {"lat": 29.949705818, "lon": 76.816615018, "alt_m": 5.0}, 
        {"lat": 29.949797999, "lon": 76.816624577, "alt_m": 5.0}, 
        {"lat": 29.949568405, "lon": 76.816471592, "alt_m": 5.0}, 
        {"lat": 29.949517131, "lon": 76.816385562, "alt_m": 5.0}, 
        {"lat": 29.949890181, "lon": 76.816634136, "alt_m": 5.0}, 
        {"lat": 29.949982362, "lon": 76.816643695, "alt_m": 5.0}, 
        {"lat": 29.949465857, "lon": 76.816299533, "alt_m": 5.0},
    ], 
    "timestamp": 1768156109.9498596
}

async def main(kml_path: str):
    kml_file = Path(kml_path)
    if not kml_file.exists():
        raise FileNotFoundError(f"KML file not found: {kml_path}")

    # Read full KML XML
    kml_xml = kml_file.read_text(encoding="utf-8")

    payload = {
        "kml_xml": kml_xml,
        "spacing": 5,    # meters
        "angle": 60      # degrees
    }

    await redis.connect()

    logger.info("Sending KML to path planner...")
    await redis.publish("mission_manager:scout_planning_request", payload)
    # await redis.client.set("path_planner:scout_waypoints", json.dumps(waypoints))
    await redis.client.set("path_planner:current_scout_waypoint_index", "0")

    # SET INTIAL VARIABLE
    await redis.client.set("path_planner:current_crop_target_index", "0")

    logger.info("Payload published successfully")

    await redis.publish("start_mission", {})

    await redis.close()

# ---------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------
def runner():
    try:
        loop.run_until_complete(main("NIDAR.kml"))
    finally:
        loop.close()

if __name__ == "__main__":
    runner()