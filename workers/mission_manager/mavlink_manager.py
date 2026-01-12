import asyncio
import json
import struct
import time
import logging
from pymavlink import mavutil
from common.redis_client import RedisClient
from workers.SETTINGS import MAVLINK_MAX_PER_TICK, MAVLINK_MAX_LATENCY_S, point_in_polygon

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

        # Track drone state for validation
        self._drone_state = {
            "scout": {
                "landed_state": 0,  # 0=UNDEFINED, 1=ON_GROUND, 2=IN_AIR
                "lat": None,
                "lon": None,
                "alt_m": None,
            },
            "sprayer": {
                "landed_state": 0,
                "lat": None,
                "lon": None,
                "alt_m": None,
            },
        }

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
        elif t == "STATUSTEXT":
            self.loop.create_task(
                self._handle_statustext(msg, drone_id)
            )
        elif t == "OBSTACLE_DISTANCE":
            self.loop.create_task(
                self._handle_obstacle_distance(msg)
            )
        elif t == "SYS_STATUS":   
            self.loop.create_task(self._handle_battery(msg, drone_id))
        elif t == "GPS_RAW_INT":
            self.loop.create_task(
                self._handle_gps_status(msg, drone_id)
            )

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

    async def _handle_statustext(self, msg, drone_id):
        """
        Handle STATUSTEXT messages from drones.
        
        Crop detection format from scout pcam.py:
        "2,<lat_int>,<lon_int>,<confidence>"
        where event_id 2 = CROP_DETECTED
        
        Sprayer finished format from sprayer_cam.py:
        "1,<result>,<spray_time_ms>,0"
        where event_id 1 = SPRAYER_FINISHED
        """
        text = msg.text.strip()
        logger.debug(f"[MAVLink] STATUSTEXT from {drone_id}: {text}")

        # Try to parse as event message (comma-separated format)
        parts = text.split(",")
        if len(parts) >= 4:
            try:
                event_id = int(parts[0])
                
                if event_id == 2:  # CROP_DETECTED from scout
                    lat = int(parts[1]) / 1e7
                    lon = int(parts[2]) / 1e7
                    confidence = float(parts[3])
                    
                    # Validate: drone must be in air
                    drone_state = self._drone_state.get(drone_id, {})
                    landed_state = drone_state.get("landed_state", 0)
                    
                    # if landed_state != 2:  # Not IN_AIR
                    #     logger.warning(
                    #         f"[MAVLink] Ignoring crop detection from {drone_id}: "
                    #         f"drone not in air (landed_state={landed_state})"
                    #     )
                    #     return
                    
                    # Validate: crop must be inside farm polygon
                    if not point_in_polygon(lat, lon):
                        logger.warning(
                            f"[MAVLink] Ignoring crop detection from {drone_id}: "
                            f"location outside farm bounds (lat={lat:.6f}, lon={lon:.6f})"
                        )
                        return
                    
                    # Valid crop detection - publish to Redis
                    payload = {
                        "drone_id": drone_id,
                        "lat": lat,
                        "lon": lon,
                        "confidence": confidence,
                        "timestamp": time.time()
                    }
                    
                    logger.info(
                        f"[MAVLink] Valid crop detected by {drone_id}: "
                        f"lat={lat:.6f}, lon={lon:.6f}, confidence={confidence:.2f}"
                    )
                    
                    await self.redis.publish("mission_manager:drone_crop_detected", payload)
                    return
                
                elif event_id == 1:  # SPRAYER_FINISHED from sprayer
                    result = int(parts[1])
                    spray_time_ms = int(parts[2])
                    
                    payload = {
                        "drone_id": drone_id,
                        "result": result,
                        "spray_time_ms": spray_time_ms,
                        "timestamp": time.time()
                    }
                    
                    logger.info(
                        f"[MAVLink] Sprayer finished: result={result}, "
                        f"spray_time={spray_time_ms}ms"
                    )
                    
                    await self.redis.publish("mission_manager:drone_sprayer_finished", payload)
                    return
                    
            except (ValueError, IndexError) as e:
                logger.debug(f"[MAVLink] Could not parse as event: {e}")
        
        # Fallback: publish as general status text
        payload = {
            "drone_id": drone_id,
            "severity": msg.severity,
            "text": text,
            "timestamp": time.time()
        }
        await self.redis.publish("mission_manager:drone_statustext", payload)

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
        # Update internal state tracking
        if drone_id in self._drone_state:
            self._drone_state[drone_id]["landed_state"] = msg.landed_state

        payload = {
            "drone_id": drone_id,
            "landed_state": msg.landed_state,
            "timestamp": time.time()
        }

        await self.redis.publish("mission_manager:drone_extended_sys_state", payload)

    async def _handle_global_position(self, msg, drone_id):
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt_m = msg.alt / 1000.0
        
        # Update internal state tracking
        if drone_id in self._drone_state:
            self._drone_state[drone_id]["lat"] = lat
            self._drone_state[drone_id]["lon"] = lon
            self._drone_state[drone_id]["alt_m"] = alt_m
        
        payload = {
            "drone_id": drone_id,
            "lat": lat,
            "lon": lon,
            "alt_m": alt_m,
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

    async def _handle_gps_status(self, msg, drone_id):
        """
        GPS_RAW_INT fields:
        fix_type:
            0: no fix
            1: no fix
            2: 2D fix
            3: 3D fix
            4: DGPS fix
            5: RTK fix
        satellites_visible: number of satellites visible
        """

        payload = {
            "drone_id": drone_id,
            "fix_type": msg.fix_type,
            "satellites_visible": msg.satellites_visible,
            "timestamp": time.time()
        }

        
        await self.redis.publish(
                "mission_manager:drone_gps_status",
                payload
            )

    async def arm_and_takeoff(self, drone_id, altitude):
        """Arm and takeoff a drone. Must be called from async context."""
        link = self.scout if drone_id == "scout" else self.sprayer
        
        logger.info(f"[MAVLink] Setting {drone_id} to GUIDED mode")
        
        # First ensure GUIDED mode
        mode_id = link.mode_mapping().get("GUIDED", 4)
        link.mav.set_mode_send(
            link.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        await asyncio.sleep(1)  # Wait for mode change
        
        logger.info(f"[MAVLink] Arming {drone_id}")
        
        # Arm the drone
        link.mav.command_long_send(
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # arm
            0, 0, 0, 0, 0, 0
        )
        await asyncio.sleep(2)  # Wait for arming
        
        logger.info(f"[MAVLink] Taking off {drone_id} to {altitude}m")
        
        # Takeoff
        link.mav.command_long_send(
            link.target_system, 
            link.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,  # pitch
            0,  # empty
            0,  # empty
            0,  # yaw angle
            0,  # lat (ignored)
            0,  # lon (ignored)
            altitude  # altitude
        )
        
        logger.info(f"[MAVLink] Takeoff command sent for {drone_id}")

    async def land_and_disarm(self, drone_id, force=False):
        """Land and disarm a drone. Must be called from async context."""
        link = self.scout if drone_id == "scout" else self.sprayer

        if force:
            # force disarm
            link.mav.command_long_send(
                link.target_system,
                link.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,      # disarm
                21196,  # force flag
                0, 0, 0, 0, 0
            )
            logger.info(f"[MAVLink] Force disarmed {drone_id}")
            return
        
        logger.info(f"[MAVLink] Landing {drone_id}")
        
        # Land
        link.mav.command_long_send(
            link.target_system, link.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )

        # Wait for landing (non-blocking)
        await asyncio.sleep(10)

        logger.info(f"[MAVLink] Disarming {drone_id}")
        
        link.mav.command_long_send(
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        
        logger.info(f"[MAVLink] {drone_id} landed and disarmed")

    async def return_to_launch(self, drone_id):
        """
        Command drone to return to launch point and land.
        Uses RTL mode which will automatically return to home and land.
        """
        link = self.scout if drone_id == "scout" else self.sprayer
        
        logger.info(f"[MAVLink] {drone_id} returning to launch")
        
        # Set RTL mode
        mode_id = link.mode_mapping().get("RTL", 6)
        link.mav.set_mode_send(
            link.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        
        logger.info(f"[MAVLink] {drone_id} RTL mode set")

        if drone_id == "scout":
            # if sprayer mission_state is idle, command sprayer to RTL as well
            sprayer_state = json.loads(await self.redis.client.get("mission:state"))["sprayer"]
            if sprayer_state.lower() == "idle":
                sprayer_mode_id = self.sprayer.mode_mapping().get("RTL", 6)
                self.sprayer.mav.set_mode_send(
                    self.sprayer.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    sprayer_mode_id
                )
                logger.info(f"[MAVLink] sprayer RTL mode set due to scout RTL")

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

    def send_spray_command(self, crop_lat: float, crop_lon: float):
        """
        Send spray command to sprayer Raspberry Pi via STATUSTEXT.
        
        Format: "0,<lat_int>,<lon_int>,0"
        Event ID 0 = SPRAY_COMMAND (tells Pi to start spray procedure)
        
        The Pi (sprayer_cam.py) will:
        1. Center over crop using HSV camera
        2. Descend to spray altitude
        3. Trigger spray relay
        4. Ascend back to cruise altitude
        5. Send SPRAYER_FINISHED (event ID 1) back
        """
        # Format: event_id,lat_int,lon_int,reserved
        EVENT_SPRAY_COMMAND = 0
        msg = f"{EVENT_SPRAY_COMMAND},{int(crop_lat * 1e7)},{int(crop_lon * 1e7)},0"
        
        self.sprayer.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            msg.encode()
        )
        
        logger.info(f"[MAVLink] Sent SPRAY_COMMAND to sprayer Pi: lat={crop_lat:.6f}, lon={crop_lon:.6f}")

    def halt(self, drone_id):
        link = self.scout if drone_id == "scout" else self.sprayer
        # link.mav.command_long_send(
        #     link.target_system,
        #     link.target_component,
        #     mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
        #     0, 0, 0, 0, 0, 0, 0, 0
        # )
        link.mav.set_position_target_local_ned_send(
            0,
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000000,  # position only
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0
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