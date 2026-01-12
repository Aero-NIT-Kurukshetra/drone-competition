import struct
from pymavlink import mavutil

def runner():
    # 🔑 INCOMING ONLY socket
    master = mavutil.mavlink_connection(
        "udp:0.0.0.0:13560"
    )

    print("Receiver listening...")

    while True:
        msg = master.recv_match(
            type="ENCAPSULATED_DATA",
            blocking=True
        )

        raw = bytes(msg.data)

        msg_type, lat_i, lon_i, conf = struct.unpack("<BiiB", raw[:10])

        lat = lat_i / 1e7
        lon = lon_i / 1e7

        print(
            f"RECV seq={msg.seqnr} "
            f"type={msg_type} "
            f"lat={lat:.6f} "
            f"lon={lon:.6f} "
            f"conf={conf}"
        )

if __name__ == "__main__":
    runner()
