"""
Scout Drone Camera Processing Script (Raspberry Pi)
====================================================

This script runs on the scout drone's Raspberry Pi and handles:
1. Continuous capture from PiCamera
2. HSV-based yellow crop detection
3. GPS-based crop geolocation
4. Sending crop detections to GCS via MAVLink STATUSTEXT

Event IDs:
- 2: CROP_DETECTED - Sent when a new crop is detected

The GCS (mission_manager) validates:
- Drone is in air (landed_state=2)
- Crop location is within farm polygon bounds

Author: NIDAR Team
"""

import cv2
import numpy as np
from pymavlink import mavutil
import math
import time
import threading
import logging

# ============================
# RASPBERRY PI IMPORTS
# ============================
try:
    from picamera2 import Picamera2
    RUNNING_ON_PI = True
except ImportError:
    RUNNING_ON_PI = False
    print("[WARNING] Running in simulation mode (not on Raspberry Pi)")

# ============================
# LOGGING SETUP
# ============================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("scout_cam")

# ============================
# CONFIG
# ============================
WORKER_ID = "camera_worker_scout"

# Camera settings
FRAME_W = 640
FRAME_H = 480

# Detection thresholds
AREA_THRESHOLD = 100           # Minimum contour area
DUPLICATE_RADIUS_M = 1.0       # Radius to consider duplicate detections

# Flight parameters
ALTITUDE_M = 5.0               # Expected flight altitude for pixel-to-meter conversion

# Camera field of view
HFOV_DEG = 62.2
VFOV_DEG = 48.8

# MAVLink connection
DRONE_INTERFACE = "/dev/ttyACM0"  # Or UDP link like "udp:localhost:14550"
BAUD_RATE = 57600

# Event IDs
EVENT_CROP_DETECTED = 2

# Processing rate limiting
DETECTION_COOLDOWN_S = 0.5     # Minimum time between detections

# ============================
# MAVLINK CONNECTION
# ============================
logger.info("Connecting to Pixhawk...")
pixhawk = mavutil.mavlink_connection(DRONE_INTERFACE, baud=BAUD_RATE)
logger.info("Waiting for heartbeat...")
pixhawk.wait_heartbeat()
logger.info(f"Connected to Pixhawk (system {pixhawk.target_system})")

# ============================
# GPS THREAD
# ============================
gps_lock = threading.Lock()
latest_gps = {
    "lat": None,
    "lon": None,
    "alt": None,
    "ts": 0.0
}

def gps_reader():
    """Background thread to continuously read GPS updates."""
    # Request all data streams
    pixhawk.mav.request_data_stream_send(
        pixhawk.target_system,
        pixhawk.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10,  # 10 Hz
        1    # Start
    )

    while True:
        try:
            msg = pixhawk.recv_match(
                type="GLOBAL_POSITION_INT",
                blocking=True,
                timeout=1.0
            )
            if msg:
                with gps_lock:
                    latest_gps["lat"] = msg.lat / 1e7
                    latest_gps["lon"] = msg.lon / 1e7
                    latest_gps["alt"] = msg.relative_alt / 1000.0
                    latest_gps["ts"] = time.time()
        except Exception as e:
            logger.error(f"GPS thread error: {e}")
            time.sleep(0.1)

gps_thread = threading.Thread(target=gps_reader, daemon=True)
gps_thread.start()
logger.info("GPS reader thread started")

def get_latest_gps():
    """Get the latest GPS position."""
    with gps_lock:
        return latest_gps["lat"], latest_gps["lon"], latest_gps["alt"]

# ============================
# CAMERA SETUP
# ============================
if RUNNING_ON_PI:
    logger.info("Initializing PiCamera2...")
    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (FRAME_W, FRAME_H), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1)  # Camera warm-up
    logger.info("Camera initialized")
else:
    # Simulation mode - use webcam if available
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        logger.info("Using webcam for simulation")
    else:
        cap = None
        logger.warning("No camera available for simulation")

# ============================
# DETECTION HELPERS
# ============================
recent_crops = []
last_detection_time = 0

def capture_frame():
    """Capture a frame from the camera."""
    if RUNNING_ON_PI:
        frame_rgb = picam2.capture_array()
        return cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    elif cap is not None:
        ret, frame = cap.read()
        return frame if ret else None
    else:
        return None

def pixel_to_ground_offset(cx, cy, altitude_m=None):
    """Convert pixel coordinates to ground offset in meters."""
    if altitude_m is None:
        altitude_m = ALTITUDE_M
        
    dx_norm = (cx - FRAME_W / 2) / (FRAME_W / 2)
    dy_norm = (cy - FRAME_H / 2) / (FRAME_H / 2)

    x_m = dx_norm * altitude_m * math.tan(math.radians(HFOV_DEG / 2))
    y_m = dy_norm * altitude_m * math.tan(math.radians(VFOV_DEG / 2))
    return x_m, y_m

def meters_to_latlon(lat, lon, dx, dy):
    dlat = dy / 111111
    dlon = dx / (111111 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon

def haversine_m(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS coordinates in meters."""
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = (
        math.sin(dphi / 2) ** 2 +
        math.cos(phi1) * math.cos(phi2) *
        math.sin(dlambda / 2) ** 2
    )
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def is_duplicate_crop(lat, lon):
    """Check if a crop at this location was recently detected."""
    for crop in recent_crops:
        if haversine_m(lat, lon, crop["lat"], crop["lon"]) < DUPLICATE_RADIUS_M:
            return True
    return False

def yellow_mask(frame_bgr):
    """Create a mask for yellow-colored crops."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    lower = np.array([20, 70, 120])
    upper = np.array([35, 255, 255])
    return cv2.inRange(hsv, lower, upper)

# ============================
# MAIN LOOP
# ============================
def main():
    """Main detection loop."""
    global last_detection_time
    
    logger.info("=" * 50)
    logger.info("Scout Camera Worker Started")
    logger.info("Detecting yellow crops and streaming to GCS...")
    logger.info("=" * 50)
    
    try:
        while True:
            loop_start = time.time()

            # Capture frame
            frame = capture_frame()
            if frame is None:
                time.sleep(0.1)
                continue

            # Apply HSV mask and find contours
            mask = yellow_mask(frame)
            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            if not contours:
                continue

            # Find largest contour
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area <= AREA_THRESHOLD:
                continue

            # Rate limit detections
            if time.time() - last_detection_time < DETECTION_COOLDOWN_S:
                continue

            # Get bounding box center
            x, y, w, h = cv2.boundingRect(c)
            cx = x + w // 2
            cy = y + h // 2

            # Get current GPS
            drone_lat, drone_lon, drone_alt = get_latest_gps()
            if drone_lat is None:
                continue

            # Use actual altitude for better accuracy
            current_alt = drone_alt if drone_alt is not None else ALTITUDE_M

            # Calculate crop position
            dx, dy = pixel_to_ground_offset(cx, cy, current_alt)
            
            # Convert offset to lat/lon
            dlat = dy / 111111
            dlon = dx / (111111 * math.cos(math.radians(drone_lat)))
            crop_lat = drone_lat + dlat
            crop_lon = drone_lon + dlon

            # Check for duplicate
            if is_duplicate_crop(crop_lat, crop_lon):
                continue

            # Calculate confidence based on area
            confidence = min(1.0, area / 5000.0)

            # Record this detection
            recent_crops.append({
                "lat": crop_lat,
                "lon": crop_lon,
                "ts": time.time()
            })
            last_detection_time = time.time()

            # Limit recent crops list size
            if len(recent_crops) > 100:
                recent_crops.pop(0)

            # Send to GCS via STATUSTEXT
            # Format: "event_id,lat_int,lon_int,confidence"
            mav_msg = f"{EVENT_CROP_DETECTED},{int(crop_lat * 1e7)},{int(crop_lon * 1e7)},{confidence:.2f}"

            pixhawk.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,
                mav_msg.encode()
            )

            logger.info(
                f"Detected crop at lat={crop_lat:.6f}, lon={crop_lon:.6f}, "
                f"confidence={confidence:.2f}, area={area:.0f}px"
            )

    except KeyboardInterrupt:
        logger.info("Shutdown requested")
    finally:
        # Cleanup
        if RUNNING_ON_PI:
            picam2.stop()
        elif cap is not None:
            cap.release()
        
        logger.info("Scout camera worker stopped")

if __name__ == "__main__":
    main()
