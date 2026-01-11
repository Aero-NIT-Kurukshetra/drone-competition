import time
import cv2
import numpy as np
from pymavlink import mavutil

# ============================================================
# MAVLINK CONNECTION
# ============================================================
mavlink = mavutil.mavlink_connection('udp:127.0.0.1:14550')
mavlink.wait_heartbeat()
print("✅ MAVLink connected (Scout Drone)")

# ============================================================
# CAMERA CONFIG
# ============================================================
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)  # DSHOW helps on Windows
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

if not cap.isOpened():
    raise RuntimeError("❌ Camera not accessible")

# ============================================================
# DETECTION CONFIG
# ============================================================
MIN_AREA = 300
SEND_INTERVAL = 0.5

last_send_time = 0.0

# ============================================================
# YELLOW DETECTION
# ============================================================
def detect_yellow_crop(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_yellow = np.array([18, 60, 60])
    upper_yellow = np.array([35, 255, 255])

    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None

    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)

    if area < MIN_AREA:
        return None

    x, y, w, h = cv2.boundingRect(c)
    cx = x + w // 2
    cy = y + h // 2

    return cx, cy, area, (x, y, w, h)

# ============================================================
# MAIN LOOP
# ============================================================
print("🚀 Scout camera processing started (DEBUG MODE)")

while True:
    ret, frame = cap.read()
    if not ret:
        time.sleep(0.05)
        continue

    detection = detect_yellow_crop(frame)

    if detection:
        cx, cy, area, bbox = detection
        x, y, w, h = bbox

        # Draw bounding box (yellow)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)

        # Draw center point (RED DOT)
        cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)

        # Compute offsets
        offset_x = (cx - FRAME_WIDTH / 2) / (FRAME_WIDTH / 2)
        offset_y = (cy - FRAME_HEIGHT / 2) / (FRAME_HEIGHT / 2)
        confidence = min(area / 2000.0, 1.0)

        now = time.time()

        if now - last_send_time >= SEND_INTERVAL:
            last_send_time = now
            ts_usec = int(now * 1e6)

            mavlink.mav.debug_vect_send(
                b"CROP_DET",
                ts_usec,
                float(offset_x),
                float(offset_y),
                float(confidence)
            )

    # Show frame
    cv2.imshow("Scout Camera - Yellow Detection", frame)

    # Exit on Q or ESC
    key = cv2.waitKey(1) & 0xFF
    if key == 27 or key == ord('q'):
        break

# ============================================================
# CLEANUP
# ============================================================
cap.release()
cv2.destroyAllWindows()
print("🛑 Scout camera stopped")
