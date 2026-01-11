import asyncio
import logging
import time
from common.redis_client import RedisClient

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("mock_camera")

WORKER_ID = "mock_camera"
loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

redis = RedisClient(loop=loop, worker_id=WORKER_ID)

async def publish_mock_gps():
    while True:
        await redis.publish("event:drone_gps_update", {
            "lat": 29.9682,
            "lon": 76.8783
        })
        logger.info("📡 Mock GPS published")
        await asyncio.sleep(1)

async def main():
    await redis.connect()
    await publish_mock_gps()

def runner():
    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        pass
    finally:
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()

if __name__ == "__main__":
    runner()
