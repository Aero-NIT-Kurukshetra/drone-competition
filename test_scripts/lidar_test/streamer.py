import time
import numpy as np
from rplidar import RPLidar
from pymavlink import mavutil

LIDAR_PORT = "COM17"          # change this
LIDAR_BAUD = 1000000          # RPLidar S2/S2L
ANGLE_RES_DEG = 5
NUM_BINS = int(360 / ANGLE_RES_DEG)

MIN_DIST_CM = 50
MAX_DIST_CM = 1500   # rplidar max limit is 18m, but we cap lower

MAVLINK_UDP = "udp:127.0.0.1:13551"

retry_count = 0
max_retries = 5

def runner():
    global retry_count
    
    master = mavutil.mavlink_connection(
        MAVLINK_UDP,
        source_system=1,
        source_component=196  # ONBOARD_COMPUTER
    )
    master.wait_heartbeat()
    print("Connected to ArduPilot")

    lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUD)
    lidar.start_motor()
    time.sleep(2)

    print("RPLidar started")

    try:
        for scan in lidar.iter_scans(max_buf_meas=7000):
            distances = np.full(NUM_BINS, MAX_DIST_CM, dtype=np.uint16)

            for (_, angle_deg, dist_mm) in scan:
                if dist_mm <= 0:
                    continue

                dist_cm = int(dist_mm / 10)
                if dist_cm < MIN_DIST_CM or dist_cm > MAX_DIST_CM:
                    continue

                idx = int(angle_deg / ANGLE_RES_DEG) % NUM_BINS
                distances[idx] = min(distances[idx], dist_cm)

            master.mav.obstacle_distance_send(
                time_usec=int(time.time() * 1e6),
                sensor_type=mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                distances=distances.tolist(),
                increment=0,                       # MUST be 0 when using increment_f
                min_distance=MIN_DIST_CM,
                max_distance=MAX_DIST_CM,
                increment_f=ANGLE_RES_DEG,         
                angle_offset=0.0,
                frame=mavutil.mavlink.MAV_FRAME_BODY_FRD
            )

            if retry_count > 0:
                retry_count = 0

            # ~8–10 Hz is ideal
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping...")

    except:
        retry_count += 1
        print(f"Error occurred. Retry {retry_count}/{max_retries}")
        if retry_count >= max_retries:
            print("Max retries reached. Exiting.")
        else:
            runner()

    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()

if __name__ == "__main__":
    runner()