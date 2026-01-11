import asyncio
import logging
import time
import math
import cv2
import numpy as np

from common.redis_client import RedisClient

# ---------------- LOGGING ----------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

logger = logging.getLogger(__name__)

# ---------------- WORKER CONFIG ----------------
WORKER_ID = "camera_worker"
CAMERA_MODE = "scout"     # "scout" or "sprayer"

loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

shutdown_event = asyncio.Event()
redis = RedisClient(loop=loop, worker_id=WORKER_ID)

# ---------------- CAMERA & GEO PARAMS ----------------
ALTITUDE = 5.0  # meters
HFOV = math.radians(78)
VFOV = math.radians(64)

REDIS_GEO_KEY = "diseased_crops"
REDIS_EVENT_CHANNEL = "event:crop_detected"

# duplicate suppression (distance only)
DUPLICATE_RADIUS_METERS = 0.5

# ---------------- STATE ----------------
cap = None
latest_gps = {
    "lat": None,
    "lon": None
}

# ---------------- REDIS LISTENERS ----------------
@redis.listen("event:drone_gps_update")
async def handle_gps_update(data):
    """
    Payload:
    {
        "lat": float,
        "lon": float
    }
    """
    latest_gps["lat"] = data.get("lat")
    latest_gps["lon"] = data.get("lon")
    logger.debug(f"[{WORKER_ID}] GPS updated: {latest_gps}")

# ---------------- VISION ----------------
def detect_yellow_leaves(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_yellow = np.array([18, 60, 60])
    upper_yellow = np.array([35, 255, 255])

    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    detections = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < 300:
            continue

        x, y, w, h = cv2.boundingRect(c)
        cx = x + w // 2
        cy = y + h // 2
        detections.append((cx, cy, area))

    return detections

def pixel_to_gps(cx, cy, img_w, img_h, lat, lon):
    ground_w = 2 * ALTITUDE * math.tan(HFOV / 2)
    ground_h = 2 * ALTITUDE * math.tan(VFOV / 2)

    dx = (cx - img_w / 2) / img_w * ground_w
    dy = (cy - img_h / 2) / img_h * ground_h

    dlat = dy / 111111
    dlon = dx / (111111 * math.cos(math.radians(lat)))

    return lat + dlat, lon + dlon

# ---------------- DUPLICATE SUPPRESSION ----------------
async def is_duplicate(lat, lon):
    """
    Returns True if another crop already exists
    within DUPLICATE_RADIUS_METERS
    """
    results = await redis.client.georadius(
        REDIS_GEO_KEY,
        lon,
        lat,
        DUPLICATE_RADIUS_METERS,
        unit="m"
    )
    return len(results) > 0

# ---------------- CAMERA LOOP ----------------
async def camera_loop():
    global cap

    cap = cv2.VideoCapture(0)  # replace with RTSP / GStreamer if needed

    if not cap.isOpened():
        logger.error(f"[{WORKER_ID}] Failed to open camera")
        shutdown_event.set()
        return

    logger.info(f"[{WORKER_ID}] Camera started in {CAMERA_MODE.upper()} mode")

    while not shutdown_event.is_set():
        ret, frame = cap.read()
        if not ret:
            await asyncio.sleep(0.05)
            continue

        if latest_gps["lat"] is None:
            await asyncio.sleep(0.05)
            continue

        h, w = frame.shape[:2]
        detections = detect_yellow_leaves(frame)

        for cx, cy, area in detections:
            lat, lon = pixel_to_gps(
                cx, cy,
                w, h,
                latest_gps["lat"],
                latest_gps["lon"]
            )

            # 🔁 Duplicate suppression (distance only)
            if await is_duplicate(lat, lon):
                logger.debug(f"[{WORKER_ID}] Duplicate crop ignored")
                continue

            # ---------------- SCOUT MODE ----------------
            if CAMERA_MODE == "scout":
                crop_id = f"crop_{int(time.time() * 1000)}"

                await redis.client.geoadd(
                    REDIS_GEO_KEY,
                    (lon, lat, crop_id)
                )

                event = {
                    "event": "CROP_DETECTED",
                    "crop_id": crop_id,
                    "gps_position": {
                        "lat": lat,
                        "lon": lon
                    },
                    "confidence": min(area / 2000.0, 1.0),
                    "source": "scout",
                    "timestamp": time.time()
                }

                await redis.publish(REDIS_EVENT_CHANNEL, event)
                logger.info(f"[{WORKER_ID}] Crop detected & saved: {crop_id}")

            # ---------------- SPRAYER MODE ----------------
            elif CAMERA_MODE == "sprayer":
                # TODO:
                # 1. Compute center offset for spray correction
                # 2. Verify yellow reduction after spray
                # 3. Publish event:spray_feedback / event:spray_completed
                pass

        await asyncio.sleep(0.05)  # ~20 FPS

# ---------------- HEARTBEAT ----------------
async def heartbeat_loop():
    while not shutdown_event.is_set():
        try:
            await redis.heartbeat()
        except Exception as e:
            logger.error(f"[{WORKER_ID}] Heartbeat error: {e}")
        await asyncio.sleep(1)

# ---------------- MAIN ----------------
async def main(loop):
    await redis.connect()

    startup_mode = await redis.get_startup_mode()
    logger.info(f"[{WORKER_ID}] Starting in {startup_mode} mode")

    tasks = [
        loop.create_task(camera_loop()),
        loop.create_task(heartbeat_loop()),
    ]

    await shutdown_event.wait()

    logger.info(f"[{WORKER_ID}] Shutting down...")
    for task in tasks:
        task.cancel()

    if cap:
        cap.release()

    await redis.close()

def handle_shutdown():
    logger.info(f"[{WORKER_ID}] Shutdown signal received")
    shutdown_event.set()

def runner():
    try:
        loop.run_until_complete(main(loop))
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"[{WORKER_ID}] Fatal error: {e}")
    finally:
        handle_shutdown()
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()

if __name__ == "__main__":
    runner()
