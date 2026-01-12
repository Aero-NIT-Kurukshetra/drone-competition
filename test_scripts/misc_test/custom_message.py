# import time
# import struct
# from pymavlink import mavutil

# def runner():
#     # 🔑 OUTGOING ONLY socket
#     master = mavutil.mavlink_connection(
#         "udpout:127.0.0.1:13560",
#         source_system=42,
#         source_component=191,
#     )

#     seq = 0

#     print("Sender started")

#     while True:
#         # Payload format:
#         # uint8  msg_type
#         # int32  lat (deg * 1e7)
#         # int32  lon (deg * 1e7)
#         # uint8  confidence
#         payload = struct.pack(
#             "<BiiB",
#             0,                              # CAMERA_MESSAGE
#             int(12.9716 * 1e7),             # lat
#             int(77.5946 * 1e7),             # lon
#             92                              # confidence
#         )

#         master.mav.encapsulated_data_send(
#             seqnr=seq,
#             data=payload.ljust(253, b"\0")  # MUST be exactly 253 bytes
#         )

#         print(f"SENT seq={seq}")

#         seq = (seq + 1) & 0xFFFF
#         time.sleep(0.5)   # 2 Hz

# if __name__ == "__main__":
#     runner()

from pymavlink import mavutil
from mav_dialect import generated_dialect
from mav_dialect.generated_dialect import MAVLink
from typing import cast
import struct
import time

master = mavutil.mavlink_connection('udp:127.0.0.1:13561')
print("Waiting for MAVLink heartbeat...")
master.wait_heartbeat()

# master.mav = generated_dialect.MAVLink(master.mav.file, master.mav.srcSystem, master.mav.srcComponent)

print("Connected to MAVLink")

def runner():
    # send sprayer start message
    while True:
        # master.mav.sprayer_command_send(
        # 	command=1,  # START
        # 	duration_ms=2000
        # )

        # # send sprayer finished message
        # master.mav.sprayer_finished_send(
        # 	result=1,           # 1 = SUCCESS
        # 	spray_time_ms=2980,
        # )

        # master.mav.crop_detected_send(
        # 	lat=12.9716,
        # 	lon=77.5946,
        # 	confidence=0.91,
        # )

        crop_payload = struct.pack(
            "<BiiB",
            0,           # CAMERA_MESSAGE
            int(12.9716 * 1e7), # lat
            int(77.5946 * 1e7), # lon
            92            # confidence
        )

        sprayer_start_payload = struct.pack(
            "<BiiB",
            1,           # SPRAYER_COMMAND_MESSAGE
            1,           # START
            2000,        # duration_ms
            0            # unused
        )

        sprayer_finished_payload = struct.pack(
            "<BiiB",
            2,           # SPRAYER_FINISHED_MESSAGE
            1,           # SUCCESS
            2980,        # spray_time_ms
            0            # unused
        )

        master.mav.encapsulated_data_send(
            seqnr=0,
            data=sprayer_finished_payload.ljust(253, b'\0')  # pad to 253 bytes
        )

        time.sleep(0.2)

        # mock OBSTACLE_DISTANCE message
        # master.mav.obstacle_distance_send(
        #     time_usec=0,
        #     sensor_type=mavutil.mavlink.MAV_DISTANCE_SENSOR_LIDAR,
        #     distances=[1000, 900, 800, 700, 600, 500, 400, 300, 200, 100],
        #     increment=15,
        #     min_distance=100,
        #     max_distance=2000,
        #     angle_offset=0,
        #     frame=mavutil.mavlink.MAV_FRAME_BODY_NED
        # )


if __name__ == "__main__":
    runner()