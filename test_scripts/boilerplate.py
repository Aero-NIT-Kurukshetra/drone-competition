import asyncio
import logging

from common.redis_client import RedisClient

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
)
logger = logging.getLogger(__name__)

loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

WORKER_ID = "test_worker"
redis = RedisClient(loop=loop, worker_id=WORKER_ID)

async def main():
    # insert test logic here 

    await redis.close()

def runner():
    try:
        loop.run_until_complete(main())
    finally:
        loop.close()

if __name__ == "__main__":
    runner()