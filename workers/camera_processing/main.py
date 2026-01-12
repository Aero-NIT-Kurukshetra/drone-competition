import asyncio
import logging
import time
import math
import cv2
import numpy as np

from common.redis_client import RedisClient
from workers.SETTINGS import (
    WORKER_ID_CAMERA,
    CAMERA_MODE,
    CAMERA_ALTITUDE,
    CAMERA_HFOV_DEG,
    CAMERA_VFOV_DEG,
    REDIS_GEO_KEY,
    REDIS_EVENT_CHANNEL,
    DUPLICATE_RADIUS_METERS,
)

# ---------------- LOGGING ----------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

logger = logging.getLogger(__name__)

# ---------------- WORKER CONFIG ----------------
WORKER_ID = WORKER_ID_CAMERA

loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

shutdown_event = asyncio.Event()
redis = RedisClient(loop=loop, worker_id=WORKER_ID)

# ---------------- CAMERA & GEO PARAMS ----------------
ALTITUDE = CAMERA_ALTITUDE
HFOV = math.radians(CAMERA_HFOV_DEG)
VFOV = math.radians(CAMERA_VFOV_DEG)

# ---------------- STATE ----------------
cap = None
latest_gps = {
    "lat": None,
    "lon": None
}

# ---------------- REDIS LISTENERS ----------------
@redis.listen("mission_manager:drone_pose_update")
async def handle_gps_update(data):
    """
    Payload from Mission Manager:
    {
        "drone_id": str,
        "lat": float,
        "lon": float,
        "alt_m": float,
        "timestamp": float
    }
    """
    # Only track scout drone GPS for crop detection
    if data.get("drone_id") != "scout":
        return
    
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

                # Publish crop detection event with flat lat/lon format
                # Mission Manager expects: {"lat": float, "lon": float, "confidence": float}
                event = {
                    "lat": lat,
                    "lon": lon,
                    "confidence": min(area / 2000.0, 1.0),
                    "crop_id": crop_id,
                    "source": "scout_camera",
                    "timestamp": time.time()
                }

                await redis.publish(REDIS_EVENT_CHANNEL, event)
                logger.info(f"[{WORKER_ID}] Crop detected & saved: {crop_id}")

            # ---------------- SPRAYER MODE ----------------
            elif CAMERA_MODE == "sprayer":
                # Compute center offset for spray correction
                # The crop should be in the center of the frame for accurate spraying
                
                # Calculate offset from image center (in pixels)
                offset_x = cx - w / 2  # positive = crop is to the right
                offset_y = cy - h / 2  # positive = crop is below center
                
                # Convert pixel offset to meters
                ground_w = 2 * ALTITUDE * math.tan(HFOV / 2)
                ground_h = 2 * ALTITUDE * math.tan(VFOV / 2)
                
                offset_m_x = (offset_x / w) * ground_w
                offset_m_y = (offset_y / h) * ground_h
                
                # Calculate distance from center
                offset_distance = math.sqrt(offset_m_x**2 + offset_m_y**2)
                
                # Publish center correction feedback
                # Mission Manager can use this to adjust sprayer position
                await redis.publish("camera:sprayer_center_offset", {
                    "offset_x_m": offset_m_x,
                    "offset_y_m": offset_m_y,
                    "offset_distance_m": offset_distance,
                    "crop_area": area,
                    "drone_lat": latest_gps["lat"],
                    "drone_lon": latest_gps["lon"],
                    "crop_lat": lat,
                    "crop_lon": lon,
                    "centered": offset_distance < 0.3,  # Within 30cm is considered centered
                    "timestamp": time.time()
                })
                
                if offset_distance < 0.3:
                    logger.info(f"[{WORKER_ID}] Crop centered (offset: {offset_distance:.2f}m)")
                else:
                    logger.debug(f"[{WORKER_ID}] Crop offset: x={offset_m_x:.2f}m, y={offset_m_y:.2f}m")

        # await asyncio.sleep(0.05)  # ~20 FPS

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
