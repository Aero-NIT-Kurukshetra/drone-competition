import asyncio
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

# ---------------------------------------------------------------------
# Main test logic
# ---------------------------------------------------------------------
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
    await redis.publish("event:pathplanner_in", payload)

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