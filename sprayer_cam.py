"""
Sprayer Drone Camera Processing Script (Raspberry Pi)
======================================================

This script runs on the sprayer drone's Raspberry Pi and handles:
1. Receiving STATUSTEXT commands from GCS via MAVLink
2. Centering over crops using HSV-based detection
3. Lowering to spray altitude and triggering the spray relay
4. Sending completion status back to GCS

Event IDs:
- 0: SPRAY_COMMAND  - GCS signals sprayer has reached crop, start centering
- 1: SPRAYER_FINISHED - Sent by this script when spraying is complete

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
    import RPi.GPIO as GPIO
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
logger = logging.getLogger("sprayer_cam")

# ============================
# CONFIG
# ============================
WORKER_ID = "camera_worker_sprayer"

# Camera settings
FRAME_W = 640
FRAME_H = 480

# Detection thresholds
AREA_THRESHOLD = 100           # Minimum contour area to consider
CENTER_TOLERANCE_PX = 30       # Pixels - when to consider centered

# Flight parameters
CRUISE_ALTITUDE_M = 5.0        # Altitude for navigation
SPRAY_ALTITUDE_M = 1.0         # Altitude for spraying
CENTER_CORRECTION_STEP_M = 0.5 # Step size for centering corrections
MAX_CENTER_ATTEMPTS = 10       # Maximum centering iterations

# Camera field of view (for pixel to meter conversion)
ALTITUDE_M = CRUISE_ALTITUDE_M
HFOV_DEG = 62.2
VFOV_DEG = 48.8

# Spray settings
SPRAY_RELAY_PIN = 17           # GPIO pin for spray relay
SPRAY_DURATION_S = 2.0         # Duration to activate spray

# MAVLink connection
DRONE_INTERFACE = "udp:localhost:13561"  # Or UDP link like "udp:localhost:14550"
BAUD_RATE = 57600

# Event IDs (matching GCS protocol)
EVENT_SPRAY_COMMAND = 0        # Received from GCS
EVENT_SPRAYER_FINISHED = 1     # Sent to GCS

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
# GPIO SETUP
# ============================
if RUNNING_ON_PI:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SPRAY_RELAY_PIN, GPIO.OUT)
    GPIO.output(SPRAY_RELAY_PIN, GPIO.LOW)
    logger.info(f"GPIO {SPRAY_RELAY_PIN} initialized for spray relay")

# ============================
# HELPER FUNCTIONS
# ============================
def capture_frame():
    """Capture a frame from the camera."""
    if RUNNING_ON_PI:
        frame_rgb = picam2.capture_array()
        return cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    elif cap is not None:
        ret, frame = cap.read()
        return frame if ret else None
    else:
        # Return a dummy frame for testing
        return np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)

def yellow_mask(frame_bgr):
    """Create a mask for yellow-colored crops."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    lower = np.array([20, 70, 120])
    upper = np.array([35, 255, 255])
    return cv2.inRange(hsv, lower, upper)

def find_crop_center(frame):
    """
    Find the center of the largest yellow contour in the frame.
    Returns (cx, cy) in pixels, or None if no crop found.
    """
    mask = yellow_mask(frame)
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None

    # Find largest contour
    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)

    if area < AREA_THRESHOLD:
        return None

    x, y, w, h = cv2.boundingRect(c)
    cx = x + w // 2
    cy = y + h // 2
    return (cx, cy)

def pixel_to_ground_offset(cx, cy, altitude_m):
    """
    Convert pixel coordinates to ground offset in meters.
    Returns (dx, dy) where positive dx is right, positive dy is forward.
    """
    dx_norm = (cx - FRAME_W / 2) / (FRAME_W / 2)
    dy_norm = (cy - FRAME_H / 2) / (FRAME_H / 2)

    x_m = dx_norm * altitude_m * math.tan(math.radians(HFOV_DEG / 2))
    y_m = dy_norm * altitude_m * math.tan(math.radians(VFOV_DEG / 2))
    return x_m, y_m

def meters_to_latlon(lat, lon, dx, dy):
    """Convert meter offsets to new lat/lon coordinates."""
    dlat = dy / 111111
    dlon = dx / (111111 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon

def send_position_command(lat, lon, alt_m):
    """Send position command to drone."""
    pixhawk.mav.set_position_target_global_int_send(
        0,
        pixhawk.target_system,
        pixhawk.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111000000,  # Position only
        int(lat * 1e7),
        int(lon * 1e7),
        alt_m,
        0, 0, 0,  # Velocity (ignored)
        0, 0, 0,  # Acceleration (ignored)
        0, 0      # Yaw (ignored)
    )

def trigger_spray(duration_s):
    """Activate the spray relay for the specified duration."""
    logger.info(f"Triggering spray for {duration_s} seconds...")
    
    if RUNNING_ON_PI:
        GPIO.output(SPRAY_RELAY_PIN, GPIO.HIGH)
        time.sleep(duration_s)
        GPIO.output(SPRAY_RELAY_PIN, GPIO.LOW)
    else:
        # Simulation - just wait
        time.sleep(duration_s)
    
    logger.info("Spray complete")

def send_sprayer_finished(result=0, spray_time_ms=0):
    """Send SPRAYER_FINISHED event to GCS via STATUSTEXT."""
    # Format: "event_id,result,spray_time_ms,reserved"
    mav_msg = f"{EVENT_SPRAYER_FINISHED},{result},{spray_time_ms},0"
    
    pixhawk.mav.statustext_send(
        mavutil.mavlink.MAV_SEVERITY_INFO,
        mav_msg.encode()
    )
    logger.info(f"Sent SPRAYER_FINISHED: result={result}, spray_time={spray_time_ms}ms")

def wait_for_altitude(target_alt_m, tolerance_m=0.3, timeout_s=30):
    """Wait until drone reaches target altitude."""
    start = time.time()
    while time.time() - start < timeout_s:
        _, _, current_alt = get_latest_gps()
        if current_alt is not None and abs(current_alt - target_alt_m) < tolerance_m:
            return True
        time.sleep(0.2)
    return False

# ============================
# MAIN SPRAY PROCEDURE
# ============================
def execute_spray_procedure():
    """
    Complete spray procedure:
    1. Center over crop using HSV detection
    2. Descend to spray altitude
    3. Trigger spray relay
    4. Ascend back to cruise altitude
    5. Send completion status
    """
    spray_start_time = time.time()
    
    # Step 1: Center over crop
    logger.info("Step 1: Centering over crop...")
    centered = False
    
    for attempt in range(MAX_CENTER_ATTEMPTS):
        frame = capture_frame()
        if frame is None:
            logger.warning("Failed to capture frame")
            time.sleep(0.2)
            continue
        
        crop_center = find_crop_center(frame)
        if crop_center is None:
            logger.warning(f"Attempt {attempt + 1}: Crop not visible")
            time.sleep(0.2)
            continue
        
        cx, cy = crop_center
        
        # Check if centered
        dx_px = cx - FRAME_W / 2
        dy_px = cy - FRAME_H / 2
        
        if abs(dx_px) < CENTER_TOLERANCE_PX and abs(dy_px) < CENTER_TOLERANCE_PX:
            logger.info("Crop centered successfully!")
            centered = True
            break
        
        # Calculate correction
        lat, lon, alt = get_latest_gps()
        if lat is None:
            logger.warning("GPS not available")
            time.sleep(0.2)
            continue
        
        current_alt = alt if alt is not None else CRUISE_ALTITUDE_M
        dx_m, dy_m = pixel_to_ground_offset(cx, cy, current_alt)
        
        # Limit correction step size
        dx_m = max(-CENTER_CORRECTION_STEP_M, min(CENTER_CORRECTION_STEP_M, dx_m))
        dy_m = max(-CENTER_CORRECTION_STEP_M, min(CENTER_CORRECTION_STEP_M, dy_m))
        
        new_lat, new_lon = meters_to_latlon(lat, lon, dx_m, dy_m)
        
        logger.info(
            f"Attempt {attempt + 1}: Offset ({dx_px:.0f}, {dy_px:.0f})px "
            f"→ correction ({dx_m:.2f}, {dy_m:.2f})m"
        )
        
        send_position_command(new_lat, new_lon, current_alt)
        time.sleep(0.5)  # Wait for drone to move
    
    if not centered:
        logger.warning("Could not center over crop after max attempts")
        # Continue anyway - spray at current position
    
    # Step 2: Descend to spray altitude
    logger.info(f"Step 2: Descending to {SPRAY_ALTITUDE_M}m...")
    lat, lon, _ = get_latest_gps()
    if lat is not None:
        send_position_command(lat, lon, SPRAY_ALTITUDE_M)
        if not wait_for_altitude(SPRAY_ALTITUDE_M):
            logger.warning("Timeout waiting for spray altitude")
    
    # Step 3: Trigger spray
    logger.info("Step 3: Spraying...")
    trigger_spray(SPRAY_DURATION_S)
    
    # Step 4: Ascend back to cruise altitude
    logger.info(f"Step 4: Ascending to {CRUISE_ALTITUDE_M}m...")
    lat, lon, _ = get_latest_gps()
    if lat is not None:
        send_position_command(lat, lon, CRUISE_ALTITUDE_M)
        if not wait_for_altitude(CRUISE_ALTITUDE_M):
            logger.warning("Timeout waiting for cruise altitude")
    
    # Step 5: Send completion status
    spray_duration_ms = int((time.time() - spray_start_time) * 1000)
    send_sprayer_finished(result=0, spray_time_ms=spray_duration_ms)
    
    logger.info(f"Spray procedure complete (total time: {spray_duration_ms}ms)")

def parse_statustext(text):
    """
    Parse STATUSTEXT message for event commands.
    Returns (event_id, data) tuple or (None, None) if not an event.
    """
    try:
        parts = text.strip().split(",")
        if len(parts) >= 1:
            event_id = int(parts[0])
            return event_id, parts[1:]
    except (ValueError, IndexError):
        pass
    return None, None

# ============================
# MAIN LOOP
# ============================
def main():
    """Main loop listening for spray commands."""
    logger.info("=" * 50)
    logger.info("Sprayer Camera Worker Started")
    logger.info("Listening for SPRAY_COMMAND (event_id=0)...")
    logger.info("=" * 50)
    
    try:
        while True:
            # Listen for both STATUSTEXT and COMMAND_LONG messages
            # STATUSTEXT: Primary method - GCS sends "0,lat,lon,0"
            # COMMAND_LONG: Backup method - MAV_CMD_USER_1
            msg = pixhawk.recv_match(
                type=["STATUSTEXT", "COMMAND_LONG"],
                blocking=True,
                timeout=1.0
            )
            
            if msg is None:
                continue
            
            msg_type = msg.get_type()
            
            # Handle STATUSTEXT messages
            if msg_type == "STATUSTEXT":
                text = msg.text
                logger.debug(f"Received STATUSTEXT: {text}")
                
                event_id, data = parse_statustext(text)
                
                if event_id == EVENT_SPRAY_COMMAND:
                    crop_lat = None
                    crop_lon = None
                    
                    # Parse crop coordinates if provided
                    if len(data) >= 2:
                        try:
                            crop_lat = int(data[0]) / 1e7
                            crop_lon = int(data[1]) / 1e7
                            logger.info(f"Target crop: lat={crop_lat:.6f}, lon={crop_lon:.6f}")
                        except (ValueError, IndexError):
                            pass
                    
                    logger.info("=" * 50)
                    logger.info("SPRAY_COMMAND received! Starting spray procedure...")
                    logger.info("=" * 50)
                    
                    execute_spray_procedure()
                    
                    logger.info("Returning to listening mode...")
            
            # Handle COMMAND_LONG messages (backup method)
            elif msg_type == "COMMAND_LONG":
                # MAV_CMD_USER_1 (31010) = Start spray command
                if msg.command == 31010:  # MAV_CMD_USER_1
                    logger.info("=" * 50)
                    logger.info("SPRAY_COMMAND via COMMAND_LONG received!")
                    logger.info("=" * 50)
                    
                    execute_spray_procedure()
                    
                    logger.info("Returning to listening mode...")
                
    except KeyboardInterrupt:
        logger.info("Shutdown requested")
    finally:
        # Cleanup
        if RUNNING_ON_PI:
            GPIO.output(SPRAY_RELAY_PIN, GPIO.LOW)
            GPIO.cleanup()
            picam2.stop()
        elif cap is not None:
            cap.release()
        
        logger.info("Sprayer camera worker stopped")

if __name__ == "__main__":
    main()
