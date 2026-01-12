import asyncio
import struct
import time
import logging
from pymavlink import mavutil
from common.redis_client import RedisClient
from workers.SETTINGS import MAVLINK_MAX_PER_TICK, MAVLINK_MAX_LATENCY_S

logger = logging.getLogger(__name__)

class MAVLinkManager:
    MAX_PER_TICK = MAVLINK_MAX_PER_TICK
    MAX_LATENCY_S = MAVLINK_MAX_LATENCY_S

    def __init__(self, redis: RedisClient, scout_uri: str, sprayer_uri: str, loop: asyncio.AbstractEventLoop = None):
        self.redis = redis
        self.loop = loop or asyncio.get_event_loop()

        self.scout = mavutil.mavlink_connection(
            scout_uri,
            source_system=10,
            source_component=1
        )
        self.sprayer = mavutil.mavlink_connection(
            sprayer_uri,
            source_system=11,
            source_component=1
        )
        
        self.scout.wait_heartbeat()
        logger.info("[MAVLink] Connected to scout")
        self.sprayer.wait_heartbeat()
        logger.info("[MAVLink] Connected to sprayer")
        
        self.drones = [self.scout, self.sprayer]

        # subscribe to all MAVLink messages
        for drone in self.drones:
            drone.mav.request_data_stream_send(
                drone.target_system,
                drone.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                10,  # 1 Hz
                1   # start
            )

            # limit speed
            for param in [b'WPNAV_SPEED', b'WPNAV_SPEED_DN', b'WPNAV_SPEED_UP']:
                drone.mav.param_set_send(
                    drone.target_system,
                    drone.target_component,
                    param,
                    75,  # cm/s
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                )

        logger.info(f"[MAVLink] Requested data stream from drones")

    # async def listen(self):
    #     """Main MAVLink receive loop"""
    #     while True:
    #         await self._poll(self.scout, "scout")
    #         await self._poll(self.sprayer, "sprayer")
    #         # await asyncio.sleep(0.01)

    async def poll(self, link, drone_id):
        while True:
            processed = 0
            while processed < self.MAX_PER_TICK:
                msg = link.recv_match(blocking=False)
                if not msg:
                    break

                processed += 1
                self._dispatch(msg, drone_id)

            # yield without sleeping
            await asyncio.sleep(0)

    def _dispatch(self, msg, drone_id):
        t = msg.get_type()
        # print(f"Received {t} from {drone_id}")

        if t == "SYSTEM_TIME":
            self._handle_system_time(msg)
            return

        if t == "GLOBAL_POSITION_INT":
            # if self._is_stale(msg):
            #     return
            
            self.loop.create_task(
                self._handle_global_position(msg, drone_id)
            )

        elif t == "HEARTBEAT":
            self.loop.create_task(
                self._handle_heartbeat(msg, drone_id)
            )

        elif t == "EXTENDED_SYS_STATE":
            self.loop.create_task(
                self._handle_extended_sys_state(msg, drone_id)
            )

        elif t == "OBSTACLE_DISTANCE":
            print("hi")
            self.loop.create_task(
                self._handle_obstacle_distance(msg)
            )
        elif t == "SYS_STATUS":   
            self.loop.create_task(self._handle_battery(msg, drone_id))

        elif t in ["SPRAYER_COMMAND", "SPRAYER_FINISHED", "CROP_DETECTED"]:
            print(f"[MAVLink] Received {t} from {drone_id}: {msg.to_dict()}")

        elif t == "ENCAPSULATED_DATA":
            raw = bytes(msg.data)
            data = struct.unpack("<BiiB", raw[:10])

            msg_type = data[0]

            if msg_type == 0:  # CROP_DETECTED_MESSAGE
                self.loop.create_task(
                    self._handle_crop_detected(data, drone_id)
                )
            elif msg_type == 2:  # SPRAYER_FINISHED_MESSAGE
                self.loop.create_task(
                    self._handle_sprayer_finished(data, drone_id)
                )

    def _handle_system_time(self, msg):
        self._boot_time_offset = (
            msg.time_unix_usec / 1e6
            - msg.time_boot_ms / 1000.0
        )

    def _is_stale(self, msg) -> bool:
        if self._boot_time_offset is None:
            return False

        sent_time = (
            self._boot_time_offset
            + msg.time_boot_ms / 1000.0
        )
        latency = time.time() - sent_time

        return latency > self.MAX_LATENCY_S
    
    async def _handle_heartbeat(self, msg, drone_id):
        """
        system_status:
            3: STANDBY              → Disarmed / on ground
            4: ACTIVE               → Armed, may be flying
            5: CRITICAL             → Low battery
            6: EMERGENCY            → Failsafe triggered
            7: POWEROFF             → Shutting down
            8: FLIGHT_TERMINATION   → Termination triggered
        """

        payload = {
            "drone_id": drone_id,
            "base_mode": msg.base_mode,
            "custom_mode": msg.custom_mode,
            "system_status": msg.system_status,
            "timestamp": time.time()
        }

        await self.redis.publish("mission_manager:drone_heartbeat", payload)

    async def _handle_crop_detected(self, msg, drone_id):
        payload = {
            "drone_id": drone_id,
            "lat": msg[1] / 1e7,
            "lon": msg[2] / 1e7,
            "confidence": msg[3],
            "timestamp": time.time()
        }

        await self.redis.publish("mission_manager:drone_crop_detected", payload)

    async def _handle_sprayer_finished(self, msg, drone_id):
        payload = {
            "drone_id": drone_id,
            "result": msg[1],
            "spray_time_ms": msg[2],
            "timestamp": time.time()
        }

        await self.redis.publish("mission_manager:drone_sprayer_finished", payload)

    async def _handle_extended_sys_state(self, msg, drone_id):
        """
        landed_state:
            0: UNDEFINED
            1: ON_GROUND
            2: IN_AIR
            3: TAKEOFF
            4: LANDING
        """

        payload = {
            "drone_id": drone_id,
            "landed_state": msg.landed_state,
            "timestamp": time.time()
        }

        await self.redis.publish("mission_manager:drone_extended_sys_state", payload)

    async def _handle_global_position(self, msg, drone_id):
        payload = {
            "drone_id": drone_id,
            # "position": [msg.x, msg.y, msg.z],
            "lat": msg.lat / 1e7,
            "lon": msg.lon / 1e7,
            "alt_m": msg.alt / 1000.0,
            "timestamp": time.time()
        }

        await self.redis.publish("mission_manager:drone_pose_update", payload)

    async def _handle_attitude(self, msg, drone_id):
        payload = {
            "drone_id": drone_id,
            "yaw": msg.yaw * 57.2958,  # rad → deg
            "timestamp": time.time()
        }

        await self.redis.publish("mission_manager:drone_attitude_update", payload)

    # async def _handle_system_time(self, msg):
    #     await self.redis.publish("mission_manager:system_time", {
    #         "time_unix_usec": msg.time_unix_usec
    #     })

    async def _handle_obstacle_distance(self, msg):
        payload = {
            "time_usec": msg.time_usec,
            "distances": list(msg.distances),
            "angle_offset": msg.angle_offset,
            "increment": msg.increment
        }

        await self.redis.publish("mission_manager:lidar_obstacle_distance", payload)

    def set_guided_mode(self):
        for link in self.drones:
            mode_id = link.mode_mapping()["GUIDED"]
            link.mav.set_mode_send(
                link.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
        logger.info(f"[MAVLink] Set drones to GUIDED mode")

    async def _handle_battery(self, msg, drone_id):
        """
        SYS_STATUS battery fields:
        voltage_battery: mV
        current_battery: cA
        battery_remaining: %
        """

        # Some FCs send -1 when data is unavailable
        if msg.voltage_battery <= 0:
            return

        payload = {
            "drone_id": drone_id,
            "voltage_v": msg.voltage_battery / 1000.0,
            "current_a": msg.current_battery / 100.0 if msg.current_battery != -1 else None,
            "remaining_pct": msg.battery_remaining,
            "timestamp": time.time()
        }

        await self.redis.publish(
            "mission_manager:drone_battery_update",
            payload
        )


    def arm_and_takeoff(self, drone_id, altitude):
        link = self.scout if drone_id == "scout" else self.sprayer

        # Arm
        link.mav.command_long_send(
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(2)

        # Takeoff
        link.mav.command_long_send(
            link.target_system, link.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )

    def land_and_disarm(self, drone_id, force=False):
        link = self.scout if drone_id == "scout" else self.sprayer

        if force:
            # force disarm
            link.mav.command_long_send(
                link.target_system,
                link.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                21196, 0, 0, 0, 0, 0, 0
            )
            return
        
        # Land
        link.mav.command_long_send(
            link.target_system, link.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )

        while True:
            msg = link.recv_match(type="LANDING_TARGET", blocking=True, timeout=5)
            if msg:
                break

        logger.info(f"[MAVLink] {drone_id} landing initiated")
        time.sleep(3)

        link.mav.command_long_send(
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )

    def send_waypoint(self, drone_id, x, y, z):
        link = self.scout if drone_id == "scout" else self.sprayer

        link.mav.set_position_target_local_ned_send(
            0,
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000000,  # position only
            x, y, z,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

    def send_waypoint_latlon(self, drone_id, lat, lon, alt_m):
        link = self.scout if drone_id == "scout" else self.sprayer

        # link.mav.mission_item_send(
        #     link.target_system,
        #     link.target_component,
        #     0,  # seq
        #     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        #     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        #     2,  # current
        #     1,  # autocontinue
        #     0, 0, 0, 0,
        #     lat,
        #     lon,
        #     alt_m
        # )

        link.mav.set_position_target_global_int_send(
            0,
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111000000,  # position only
            int(lat * 1e7),                       # lat_int
            int(lon * 1e7),                       # lon_int
            alt_m,                                # meters above home
            0, 0, 0,                              # velocity ignored
            0, 0, 0,                              # acceleration ignored
            0, 0                                 # yaw ignored
        )

    def halt(self, drone_id):
        link = self.scout if drone_id == "scout" else self.sprayer
        link.mav.command_long_send(
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            0, 0, 0, 0, 0, 0, 0, 0
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

        mav_manager = MAVLinkManager(redis,
            scout_uri=os.environ.get("SCOUT_MAVLINK_UDP", "udp:localhost:13550"),
            sprayer_uri=os.environ.get("SPRAYER_MAVLINK_UDP", "udp:localhost:13560")
        )

        await mav_manager.listen()

    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        print("Shutting down...")
          
if __name__ == "__main__":
    test()