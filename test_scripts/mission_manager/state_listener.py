import asyncio
import logging

from jsondiff import symbols
import jsondiff

from common.redis_client import RedisClient

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
)
logger = logging.getLogger(__name__)

loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)
shutdown_event = asyncio.Event()

WORKER_ID = "test_worker"
redis = RedisClient(loop=loop, worker_id=WORKER_ID)

last_state = None
IGNORE_KEYS = {"timestamp", "last_pose_ts"}


def print_state_diff(diff, prev_state, path=None):
    if path is None:
        path = []

    for key, value in diff.items():
        if key in IGNORE_KEYS:
            continue

        # Recurse into nested dicts
        if isinstance(value, dict):
            print_state_diff(value, prev_state, path + [key])
            continue

        # Leaf change
        old_val = prev_state
        for p in path + [key]:
            old_val = old_val.get(p, None) if isinstance(old_val, dict) else None

        # Extract drone name
        drone = path[1] if len(path) >= 2 and path[0] == "drones" else "unknown"

        print(f"[{drone}] {key}: {old_val} → {value}")

@redis.listen("mission:state_update")
async def handle_state_update(message):
    global last_state

    if last_state:
        diff = jsondiff.diff(last_state, message)
        print_state_diff(diff, last_state)

    last_state = message

async def main(loop):
    await redis.connect()

    startup_mode = await redis.get_startup_mode()
    logger.info(f"[{WORKER_ID}] Starting in {startup_mode} mode")

    # since this is a dummy worker, we just log the mode
    # but in real worker if theres a different behaviour for recovery mode, tasks[] would be adjusted accordingly
    tasks = [
        loop.create_task(heartbeat_loop()),
    ]

    await shutdown_event.wait()

    logger.info(f"[{WORKER_ID}] Shutting down...")
    for task in tasks:
        task.cancel()

    await redis.close()


async def heartbeat_loop():
    while not shutdown_event.is_set():
        try:
            await redis.heartbeat()
        except Exception as e:
            logger.error(f"Heartbeat error: {e}")
        await asyncio.sleep(1)

def handle_shutdown():
    logger.info("Shutdown signal received")
    shutdown_event.set()

def runner():
    # Handle SIGINT / SIGTERM (PM2 compatible)
    # this is commented out to avoid issues on Windows
    # for sig in (signal.SIGINT, signal.SIGTERM):
    #     loop.add_signal_handler(sig, handle_shutdown)

    try:
        loop.run_until_complete(main(loop))
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"Error in main loop: {e}")
    finally:
        handle_shutdown()
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()

if __name__ == "__main__":
    runner()