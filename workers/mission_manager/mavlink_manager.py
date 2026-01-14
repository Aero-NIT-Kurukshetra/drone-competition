import asyncio
import json
import struct
import time
import logging
from pymavlink import mavutil
from common.redis_client import RedisClient
from workers.SETTINGS import (
    MAVLINK_MAX_PER_TICK,
    MAVLINK_MAX_LATENCY_S,
    point_in_polygon,
)

logger = logging.getLogger(__name__)

SYSID_SCOUT = 1
SYSID_SPRAYER = 2
AUTOPILOT_COMPID = 1


class MAVLinkManager:
    MAX_PER_TICK = MAVLINK_MAX_PER_TICK
    MAX_LATENCY_S = MAVLINK_MAX_LATENCY_S

    def __init__(
        self,
        redis: RedisClient,
        mavlink_uri: str,
        loop: asyncio.AbstractEventLoop = None,
    ):
        self.redis = redis
        self.loop = loop or asyncio.get_event_loop()
        self._boot_time_offset = None

        # sysid ↔ drone_id mapping
        self.sysid_to_drone = {
            SYSID_SCOUT: "scout",
            SYSID_SPRAYER: "sprayer",
        }
        self.drone_to_sysid = {
            "scout": SYSID_SCOUT,
            "sprayer": SYSID_SPRAYER,
        }

        # Internal state tracking (unchanged semantics)
        self._drone_state = {
            "scout": {"landed_state": 0, "lat": None, "lon": None, "alt_m": None},
            "sprayer": {"landed_state": 0, "lat": None, "lon": None, "alt_m": None},
        }

        # Single MAVLink connection
        self.link = mavutil.mavlink_connection(
            mavlink_uri,
            source_system=255,
            source_component=1,
        )

        self.link.wait_heartbeat()
        logger.info("[MAVLink] Connected to MAVLink stream")

        # Request streams + set params per vehicle
        for sysid in self.sysid_to_drone:
            self.link.mav.request_data_stream_send(
                sysid,
                AUTOPILOT_COMPID,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                10,
                1,
            )

            for param in [b"WPNAV_SPEED", b"WPNAV_SPEED_DN", b"WPNAV_SPEED_UP"]:
                self.link.mav.param_set_send(
                    sysid,
                    AUTOPILOT_COMPID,
                    param,
                    75,
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                )

        logger.info("[MAVLink] Data streams requested")

    # ------------------------------------------------------------------
    # MAIN POLL LOOP (START ONCE)
    # ------------------------------------------------------------------

    async def poll(self):
        while True:
            processed = 0
            while processed < self.MAX_PER_TICK:
                msg = self.link.recv_match(blocking=False)
                if not msg:
                    break

                processed += 1

                sysid = msg.get_srcSystem()
                drone_id = self.sysid_to_drone.get(sysid)
                if not drone_id:
                    continue

                self._dispatch(msg, drone_id)

            await asyncio.sleep(0)

    # ------------------------------------------------------------------
    # DISPATCH
    # ------------------------------------------------------------------

    def _dispatch(self, msg, drone_id):
        t = msg.get_type()

        if t == "SYSTEM_TIME":
            self._handle_system_time(msg)
            return

        if t == "GLOBAL_POSITION_INT":
            self.loop.create_task(self._handle_global_position(msg, drone_id))

        elif t == "HEARTBEAT":
            self.loop.create_task(self._handle_heartbeat(msg, drone_id))

        elif t == "EXTENDED_SYS_STATE":
            self.loop.create_task(self._handle_extended_sys_state(msg, drone_id))

        elif t == "STATUSTEXT":
            self.loop.create_task(self._handle_statustext(msg, drone_id))

        elif t == "OBSTACLE_DISTANCE":
            self.loop.create_task(self._handle_obstacle_distance(msg))

        elif t == "SYS_STATUS":
            self.loop.create_task(self._handle_battery(msg, drone_id))

        elif t == "GPS_RAW_INT":
            self.loop.create_task(self._handle_gps_status(msg, drone_id))

        elif t == "ENCAPSULATED_DATA":
            raw = bytes(msg.data)
            data = struct.unpack("<BiiB", raw[:10])
            if data[0] == 0:
                self.loop.create_task(self._handle_crop_detected(data, drone_id))
            elif data[0] == 2:
                self.loop.create_task(self._handle_sprayer_finished(data, drone_id))

    # ------------------------------------------------------------------
    # TIME / STALENESS
    # ------------------------------------------------------------------

    def _handle_system_time(self, msg):
        self._boot_time_offset = (
            msg.time_unix_usec / 1e6 - msg.time_boot_ms / 1000.0
        )

    # ------------------------------------------------------------------
    # HANDLERS (LOGIC UNCHANGED)
    # ------------------------------------------------------------------

    async def _handle_heartbeat(self, msg, drone_id):
        await self.redis.publish(
            "mission_manager:drone_heartbeat",
            {
                "drone_id": drone_id,
                "base_mode": msg.base_mode,
                "custom_mode": msg.custom_mode,
                "system_status": msg.system_status,
                "timestamp": time.time(),
            },
        )

    async def _handle_extended_sys_state(self, msg, drone_id):
        self._drone_state[drone_id]["landed_state"] = msg.landed_state
        await self.redis.publish(
            "mission_manager:drone_extended_sys_state",
            {
                "drone_id": drone_id,
                "landed_state": msg.landed_state,
                "timestamp": time.time(),
            },
        )

    async def _handle_global_position(self, msg, drone_id):
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt_m = msg.alt / 1000.0

        self._drone_state[drone_id].update(
            {"lat": lat, "lon": lon, "alt_m": alt_m}
        )

        await self.redis.publish(
            "mission_manager:drone_pose_update",
            {
                "drone_id": drone_id,
                "lat": lat,
                "lon": lon,
                "alt_m": alt_m,
                "timestamp": time.time(),
            },
        )

    async def _handle_statustext(self, msg, drone_id):
        text = msg.text.strip()
        parts = text.split(",")

        if len(parts) >= 4:
            try:
                event_id = int(parts[0])

                if event_id == 2:  # CROP_DETECTED
                    lat = int(parts[1]) / 1e7
                    lon = int(parts[2]) / 1e7
                    confidence = float(parts[3])

                    if not point_in_polygon(lat, lon):
                        return

                    await self.redis.publish(
                        "mission_manager:drone_crop_detected",
                        {
                            "drone_id": drone_id,
                            "lat": lat,
                            "lon": lon,
                            "confidence": confidence,
                            "timestamp": time.time(),
                        },
                    )
                    return

                if event_id == 1:  # SPRAYER_FINISHED
                    await self.redis.publish(
                        "mission_manager:drone_sprayer_finished",
                        {
                            "drone_id": drone_id,
                            "result": int(parts[1]),
                            "spray_time_ms": int(parts[2]),
                            "timestamp": time.time(),
                        },
                    )
                    return
            except Exception:
                pass

        await self.redis.publish(
            "mission_manager:drone_statustext",
            {
                "drone_id": drone_id,
                "severity": msg.severity,
                "text": text,
                "timestamp": time.time(),
            },
        )

    async def _handle_crop_detected(self, msg, drone_id):
        await self.redis.publish(
            "mission_manager:drone_crop_detected",
            {
                "drone_id": drone_id,
                "lat": msg[1] / 1e7,
                "lon": msg[2] / 1e7,
                "confidence": msg[3],
                "timestamp": time.time(),
            },
        )

    async def _handle_sprayer_finished(self, msg, drone_id):
        await self.redis.publish(
            "mission_manager:drone_sprayer_finished",
            {
                "drone_id": drone_id,
                "result": msg[1],
                "spray_time_ms": msg[2],
                "timestamp": time.time(),
            },
        )

    async def _handle_battery(self, msg, drone_id):
        if msg.voltage_battery <= 0:
            return

        await self.redis.publish(
            "mission_manager:drone_battery_update",
            {
                "drone_id": drone_id,
                "voltage_v": msg.voltage_battery / 1000.0,
                "current_a": (
                    msg.current_battery / 100.0
                    if msg.current_battery != -1
                    else None
                ),
                "remaining_pct": msg.battery_remaining,
                "timestamp": time.time(),
            },
        )

    async def _handle_gps_status(self, msg, drone_id):
        await self.redis.publish(
            "mission_manager:drone_gps_status",
            {
                "drone_id": drone_id,
                "fix_type": msg.fix_type,
                "satellites_visible": msg.satellites_visible,
                "timestamp": time.time(),
            },
        )

    async def _handle_obstacle_distance(self, msg):
        await self.redis.publish(
            "mission_manager:lidar_obstacle_distance",
            {
                "time_usec": msg.time_usec,
                "distances": list(msg.distances),
                "angle_offset": msg.angle_offset,
                "increment": msg.increment,
            },
        )

    # ------------------------------------------------------------------
    # COMMAND HELPERS (INTERFACE UNCHANGED)
    # ------------------------------------------------------------------

    def _sysid(self, drone_id):
        return self.drone_to_sysid[drone_id]

    async def arm_and_takeoff(self, drone_id, altitude):
        sysid = self._sysid(drone_id)

        mode_id = self.link.mode_mapping().get("GUIDED", 4)
        self.link.mav.set_mode_send(
            sysid,
            AUTOPILOT_COMPID,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        await asyncio.sleep(1)

        self.link.mav.command_long_send(
            sysid,
            AUTOPILOT_COMPID,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        await asyncio.sleep(2)

        self.link.mav.command_long_send(
            sysid,
            AUTOPILOT_COMPID,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            altitude,
        )

    async def return_to_launch(self, drone_id):
        sysid = self._sysid(drone_id)
        mode_id = self.link.mode_mapping().get("RTL", 6)
        self.link.mav.set_mode_send(
            sysid,
            AUTOPILOT_COMPID,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

    def send_spray_command(self, crop_lat, crop_lon):
        msg = f"0,{int(crop_lat*1e7)},{int(crop_lon*1e7)},0"
        self.link.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            msg.encode(),
        )

# testing
def test():
    import os
    from dotenv import load_dotenv

    load_dotenv()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    redis = RedisClient(loop=loop)

    async def main():
        await redis.connect()

        mav_manager = MAVLinkManager(
            redis,
            scout_uri=os.environ.get("SCOUT_MAVLINK_UDP", "udp:localhost:14540"),
            sprayer_uri=os.environ.get("SPRAYER_MAVLINK_UDP", "udp:localhost:13560"),
            loop=loop
        )

        mav_manager.start()

        while True:
            await asyncio.sleep(1)


    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        print("Shutting down...")
          
if __name__ == "__main__":
    test()