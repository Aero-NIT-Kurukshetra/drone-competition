import cv2
import numpy as np
from pymavlink import mavutil
import math
import time
import threading

# ============================
# CONFIG
# ============================
WORKER_ID = "camera_worker_scout"

FRAME_W = 640
FRAME_H = 480

AREA_THRESHOLD = 500
DUPLICATE_RADIUS_M = 1.0

ALTITUDE_M = 5.0
HFOV_DEG = 62.2
VFOV_DEG = 48.8

DRONE_INTERFACE = "/dev/ttyACM0" # "udpin:localhost:13561"

pixhawk = mavutil.mavlink_connection(DRONE_INTERFACE, baud = 57600)
print("Waiting for heartbeat...")
pixhawk.wait_heartbeat()
print("Connected to Pixhawk")

gps_lock = threading.Lock()
latest_gps = {
    "lat": None,
    "lon": None,
    "ts": 0.0
}

def gps_reader():
    while True:
        try:
            msg = pixhawk.recv_match(
                type="GLOBAL_POSITION_INT",
                blocking=True
            )
            if msg:
                with gps_lock:
                    latest_gps["lat"] = msg.lat / 1e7
                    latest_gps["lon"] = msg.lon / 1e7
                    latest_gps["ts"] = time.time()
        except Exception as e:
            print("GPS thread error:", e)
            time.sleep(0.1)

gps_thread = threading.Thread(
    target=gps_reader,
    daemon=True
)
gps_thread.start()

def get_latest_gps():
    with gps_lock:
        return latest_gps["lat"], latest_gps["lon"]

recent_crops = []

def pixel_to_ground_offset(cx, cy):
    dx_norm = (cx - FRAME_W / 2) / (FRAME_W / 2)
    dy_norm = (cy - FRAME_H / 2) / (FRAME_H / 2)

    x_m = dx_norm * ALTITUDE_M * math.tan(math.radians(HFOV_DEG / 2))
    y_m = dy_norm * ALTITUDE_M * math.tan(math.radians(VFOV_DEG / 2))
    return x_m, y_m

def meters_to_latlon(lat, lon, dx, dy):
    dlat = dy / 111111
    dlon = dx / (111111 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon

def haversine_m(lat1, lon1, lat2, lon2):
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
    for crop in recent_crops:
        if haversine_m(lat, lon, crop["lat"], crop["lon"]) < DUPLICATE_RADIUS_M:
            return True
    return False

def yellow_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([20, 70, 120])
    upper = np.array([35, 255, 255])
    return cv2.inRange(hsv, lower, upper)

def centroid(contour):
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return -1, -1
    return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

cap = cv2.VideoCapture(0)

length = 0

while True:
    loop_start = time.time()

    ret, frame = cap.read()
    if not ret:
        continue

    frame = cv2.resize(frame, (FRAME_W, FRAME_H))

    mask = yellow_mask(frame)
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    print(f"Detected {len(contours)} contours")

    if contours:
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        if area > AREA_THRESHOLD:
            x, y, w, h = cv2.boundingRect(c)
            cx = x + w // 2
            cy = y + h // 2

            dx, dy = pixel_to_ground_offset(cx, cy)
            drone_lat, drone_lon = get_latest_gps()
            if drone_lat is None:
                continue  # GPS not ready
            crop_lat, crop_lon = meters_to_latlon(
                drone_lat, drone_lon, dx, dy
            )

            if not is_duplicate_crop(crop_lat, crop_lon):
                ts = time.time()
                confidence = min(1.0, area / 5000.0)
                crop_detected_event = 2

                recent_crops.append({
                    "lat": crop_lat,
                    "lon": crop_lon
                })

                mav_msg = (
                    f"{crop_detected_event},{int(crop_lat * 1e7)},{int(crop_lon * 1e7)},{confidence:.2f}"
                )

                pixhawk.mav.statustext_send(
                    mavutil.mavlink.MAV_SEVERITY_INFO,
                    mav_msg.encode()
                )

            cv2.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                (0, 255, 0),
                2
            )
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

    fps = 1.0 / (time.time() - loop_start)
    cv2.putText(
        frame,
        f"FPS: {fps:.2f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 0, 0),
        2
    )

    cv2.imshow("Leaf Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
