import asyncio
import json
import logging
import math
import os
import time
from typing import Dict, Any
from dotenv import load_dotenv
from pymavlink import mavutil

load_dotenv()

from common.redis_client import RedisClient
from workers.mission_manager.mavlink_manager import MAVLinkManager
from workers.SETTINGS import (
    WORKER_ID_MISSION_MANAGER,
    DRONES,
    STATE_REDIS_KEY,
    STATE_REDIS_DRONE_KEY,
    STATE_PUBLISH_CHANNEL,
    DEFAULT_ALTITUDE,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

logger = logging.getLogger("MissionManager")

WORKER_ID = WORKER_ID_MISSION_MANAGER

loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

shutdown_event = asyncio.Event()
redis = RedisClient(loop=loop, worker_id=WORKER_ID)

mav: MAVLinkManager | None = None

def default_drone_state():
    return {
        "mode": "IDLE",            # FSM state (NAVIGATING, HALTED, SPRAYING, etc.)
        "active_waypoint": None,   # dict or None
        "halted": False,
        "last_pose_ts": None,# timestamp of last pose update
        "current_loc": {
            "lat": None,
            "lon": None,
            "alt": DEFAULT_ALTITUDE
            }
    }

mission_state: Dict[str, Any] = {
    "system_mode": "NORMAL",
    "drones": {
        "scout": default_drone_state(),
        "sprayer": default_drone_state(),
    }
}

async def sync_state_to_redis():
    snapshot = {
        "system_mode": mission_state["system_mode"],
        "timestamp": time.time(),
        "drones": mission_state["drones"],
    }

    await redis.client.set(
        STATE_REDIS_KEY,
        json.dumps(snapshot)
    )

    for drone_id, state in mission_state["drones"].items():
        await redis.client.set(
            STATE_REDIS_DRONE_KEY.format(drone_id),
            json.dumps(state)
        )

    # Notify UI (pub/sub)
    await redis.publish(STATE_PUBLISH_CHANNEL, snapshot)

def set_mode(drone_id: str, mode: str):
    mission_state["drones"][drone_id]["mode"] = mode
    loop.create_task(sync_state_to_redis())

def halt_drone(drone_id: str):
    mav.halt(drone_id)
    d = mission_state["drones"][drone_id]
    d["halted"] = True
    d["active_waypoint"] = None
    d["mode"] = "HALTED"
    loop.create_task(sync_state_to_redis())

def resume_drone(drone_id: str):
    d = mission_state["drones"][drone_id]
    d["halted"] = False
    d["mode"] = "IDLE"
    loop.create_task(sync_state_to_redis())

def set_active_waypoint(drone_id: str, wp: dict):
    d = mission_state["drones"][drone_id]
    d["active_waypoint"] = wp
    d["mode"] = "NAVIGATING_TO_WAYPOINT"
    loop.create_task(sync_state_to_redis())

def clear_active_waypoint(drone_id: str):
    d = mission_state["drones"][drone_id]
    d["active_waypoint"] = None
    d["mode"] = "WAITING_FOR_NEXT_WAYPOINT"
    loop.create_task(sync_state_to_redis())

@redis.listen("start_mission")
async def handle_start_mission(_):
    logger.info("[MissionManager] Starting mission")

    mission_state["system_mode"] = "NORMAL"
    mission_state["drones"] = {
        "scout": default_drone_state(),
        "sprayer": default_drone_state(),
    }
    await sync_state_to_redis()

    for drone_id in DRONES:
        resume_drone(drone_id)

    mav.set_guided_mode()
    await asyncio.sleep(2)

    mav.arm_and_takeoff("scout", 5)
    set_mode("scout", "TAKING_OFF")

    await redis.publish(
        "mission_manager:request_next_waypoint",
        {"drone_id": "scout"}
    )

@redis.listen("path_planning:arm_takeoff")
async def handle_arm_takeoff(data):
    drone_id = data["drone_id"]
    altitude = data.get("altitude", 5)

    logger.info(f"[MissionManager] Arming and takeoff for {drone_id} to {altitude}m")

    mav.arm_and_takeoff(drone_id, altitude)

@redis.listen("path_planning:planned_waypoint")
async def handle_planned_waypoint(data):
    drone_id = data["drone_id"]
    waypoint = data["waypoint"]

    drone = mission_state["drones"][drone_id]

    if mission_state["system_mode"] == "RECOVERY":
        logger.warning(f"[MissionManager] Ignoring waypoint for {drone_id} (RECOVERY)")
        return

    if drone["halted"]:
        logger.warning(f"[MissionManager] Ignoring waypoint for {drone_id} (HALTED)")
        return
    
    if waypoint is None:
        logger.info(f"[MissionManager] No more waypoints for {drone_id}")
        mav.halt(drone_id)
        if drone_id == "scout":
            # change mode to LANDING
            set_mode(drone_id, "LANDING")
            mav.land_and_disarm(drone_id)
            # land 
            pass
        else:
            set_mode(drone_id, "HOVERING")

        return

    logger.info(f"[MissionManager] Executing waypoint for {drone_id}: {waypoint}")

    # TODO
    """
    For sprayer:
    - check if drone is landed or in air
    - if landed, takeoff to a safe altitude (e.g., 5m)
    - then send waypoint
    """

    mav.send_waypoint_latlon(
        drone_id,
        waypoint["lat"],
        waypoint["lon"],
        waypoint.get("alt_m", 5.0)
    )

    set_active_waypoint(drone_id, waypoint)

@redis.listen("event:no_safe_path")
async def handle_no_safe_path(data):
    drone_id = data["drone_id"]

    logger.warning(f"[MissionManager] NO SAFE PATH for {drone_id} → halting")

    mav.halt(drone_id)
    mission_state["halted"][drone_id] = True

@redis.listen("lidar_processing:occupancy_grid_updated")
async def handle_grid_update(_):
    # Any grid update invalidates current assumptions
    for drone_id in ["scout", "sprayer"]:
        if not mission_state["halted"][drone_id]:
            await redis.publish(
                "mission_manager:request_next_waypoint",
                {"drone_id": drone_id}
            )

@redis.listen("event:crop_detected")
async def handle_crop_detected(data):
    logger.info("[MissionManager] Crop detected → dispatch sprayer")

    # Example: data coming from vision worker
    # data = {
    #   "lat": 29.94783,
    #   "lon": 76.81421,
    #   "alt": 3.0
    # }

    crop_location = {
        "lat": data["lat"],
        "lon": data["lon"],
        "alt": data.get("alt", 3.0),
    }

    # Fetch current crop_locations array
    crop_locations_json = await redis.client.get("crop_locations")
    
    if crop_locations_json:
        crop_locations = json.loads(crop_locations_json)
    else:
        crop_locations = []
        # Initialize index to -1 if first crop
        await redis.client.set("path_planner:current_crop_target_index", "-1")
    
    # Append new crop location
    crop_locations.append(crop_location)
    await redis.client.set("crop_locations", json.dumps(crop_locations))

    logger.info(
        f"[MissionManager] Crop appended → "
        f"lat={crop_location['lat']}, lon={crop_location['lon']} "
        f"(total: {len(crop_locations)})"
    )

    # If sprayer is idle, dispatch it immediately
    if mission_state["mode"]["sprayer"] == "IDLE":
        await dispatch_sprayer_to_next_crop()


async def dispatch_sprayer_to_next_crop():
    """Increment crop index and request path planning for sprayer to next crop."""
    # Fetch crop_locations array
    crop_locations_json = await redis.client.get("crop_locations")
    
    if not crop_locations_json:
        logger.info("[MissionManager] No crops detected yet, sprayer remains IDLE")
        return
    
    crop_locations = json.loads(crop_locations_json)
    
    # Get current index and increment
    current_index = await redis.client.get("path_planner:current_crop_target_index")
    current_index = int(current_index) if current_index else -1
    
    next_index = current_index + 1
    
    # Check if there are more crops to spray
    if next_index >= len(crop_locations):
        logger.info(f"[MissionManager] All crops sprayed ({len(crop_locations)} total), sprayer remains IDLE")
        return
    
    # Update index in Redis
    await redis.client.set("path_planner:current_crop_target_index", str(next_index))
    
    crop = crop_locations[next_index]
    logger.info(f"[MissionManager] Dispatching sprayer to crop {next_index + 1}/{len(crop_locations)} at lat={crop['lat']}, lon={crop['lon']}")
    
    mission_state["mode"]["sprayer"] = "NAVIGATING"
    mission_state["current_target"]["sprayer"] = crop
    
    await redis.publish(
        "mission_manager:request_next_waypoint",
        {
            "drone_id": "sprayer",
            # "target": data["location"]
        }
    )

@redis.listen("mission_manager:drone_pose_update")
async def handle_pose_update(data):
    drone_id = data["drone_id"]
    drone = mission_state["drones"][drone_id]

    drone["last_pose_ts"] = data["timestamp"]

    if drone["halted"] or drone["mode"] not in ["NAVIGATING_TO_WAYPOINT", "WAITING_FOR_NEXT_WAYPOINT", "TAKING_OFF"]:
        return

    (
        scout_waypoints,
        scout_current_wp_index,
        sprayer_waypoints,
        sprayer_current_wp_index,
    ) = await redis.client.mget(
        "path_planner:scout_waypoints",
        "path_planner:current_scout_waypoint_index",
        "path_planner:sprayer_waypoints",
        "path_planner:current_sprayer_waypoint_index",
    )

    if drone_id == "scout" and scout_waypoints:
        waypoints_data = json.loads(scout_waypoints)
        scout_waypoints = waypoints_data["waypoints"]
        scout_current_wp_index = int(scout_current_wp_index)

        wp = None
        if scout_current_wp_index < len(scout_waypoints):
            wp = scout_waypoints[scout_current_wp_index]

        if not wp:
            return

        try:
            cur_lat = data["lat"]
            cur_lon = data["lon"]
        except KeyError:
            print(data)

        distance = haversine_distance(cur_lat, cur_lon, wp["lat"], wp["lon"])

        logger.info(f"[MissionManager] {drone_id} distance to waypoint: {distance:.2f}m")

        if distance < 1.0:  # 1.0m radius
            logger.info(f"[MissionManager] {drone_id} reached waypoint")

            clear_active_waypoint(drone_id)

            await redis.publish(
                "mission_manager:request_next_waypoint",
                {"drone_id": drone_id, "lat": cur_lat, "lon": cur_lon}
            )

    elif drone_id == "sprayer" and sprayer_waypoints:
        mission_state["drones"][drone_id]["current_loc"]["lat"] = data["lat"]
        mission_state["drones"][drone_id]["current_loc"]["lon"] = data["lon"]

        waypoints_data = json.loads(sprayer_waypoints)
        sprayer_waypoints = waypoints_data["waypoints"]
        sprayer_current_wp_index = int(sprayer_current_wp_index)

        if sprayer_current_wp_index >= len(sprayer_waypoints):
            return

        wp = sprayer_waypoints[sprayer_current_wp_index]

        try:
            cur_lat = data["lat"]
            cur_lon = data["lon"]
            cur_alt = data["alt_m"]
        except KeyError:
            return

        distance = haversine_distance(cur_lat, cur_lon, wp["lat"], wp["lon"])
        logger.info(f"[MissionManager] sprayer distance to waypoint: {distance:.2f}m")

        CRUISE_ALT = 5.0
        SPRAY_ALT = 1.0
        WAYPOINT_RADIUS = 1.0
        CROP_RADIUS = 0.5

        is_last_waypoint = sprayer_current_wp_index == len(sprayer_waypoints) - 1

        # ──────────────────────────────
        # 1. Normal waypoint navigation
        # ──────────────────────────────
        if not is_last_waypoint:
            if distance < WAYPOINT_RADIUS:
                logger.info("[MissionManager] sprayer reached waypoint")

                clear_active_waypoint("sprayer")

                await redis.client.incr("path_planner:current_sprayer_waypoint_index")

                await redis.publish(
                    "mission_manager:request_next_waypoint",
                    {
                        "drone_id": "sprayer",
                        "lat": cur_lat,
                        "lon": cur_lon
                    }
                )
            return

        # ──────────────────────────────
        # 2. Crop waypoint reached
        # ──────────────────────────────
        if drone["mode"] == "NAVIGATING_TO_WAYPOINT" and distance < CROP_RADIUS:
            logger.info("[MissionManager] sprayer reached crop → descending")

            drone["mode"] = "DESCENDING_TO_SPRAY"
            await sync_state_to_redis()

            mav.send_waypoint_latlon(
                "sprayer",
                wp["lat"],
                wp["lon"],
                SPRAY_ALT
            )
            return

        # ──────────────────────────────
        # 3. At spray altitude → spray
        # ──────────────────────────────
        if drone["mode"] == "DESCENDING_TO_SPRAY":
            if abs(cur_alt - SPRAY_ALT) < 0.3:
                logger.info("[MissionManager] sprayer at spray altitude → START_SPRAYING")

                drone["mode"] = "SPRAYING"
                await sync_state_to_redis()

                # Custom MAVLink command (no args)
                mav.sprayer.mav.command_long_send(
                    mav.sprayer.target_system,
                    mav.sprayer.target_component,
                    mavutil.mavlink.MAV_CMD_USER_1,  # START_SPRAYING
                    0, 0, 0, 0, 0, 0, 0, 0
                )
            return

        # ──────────────────────────────
        # 4. After spraying → ascend
        # (sprayer auto-stops after preset time)
        # ──────────────────────────────
        if drone["mode"] == "SPRAYING":
            logger.info("[MissionManager] spraying finished → ascending")

            drone["mode"] = "ASCENDING_AFTER_SPRAY"
            await sync_state_to_redis()

            mav.send_waypoint_latlon(
                "sprayer",
                wp["lat"],
                wp["lon"],
                CRUISE_ALT
            )
            return

        # ──────────────────────────────
        # 5. Back to cruise altitude → next crop
        # ──────────────────────────────
        if drone["mode"] == "ASCENDING_AFTER_SPRAY":
            if abs(cur_alt - CRUISE_ALT) < 0.3:
                logger.info("[MissionManager] sprayer ready for next crop")

                clear_active_waypoint("sprayer")
                await redis.client.incr("path_planner:current_crop_target_index")

                await redis.publish(
                    "mission_manager:request_next_waypoint",
                    {"drone_id": "sprayer"}
                )

@redis.listen("system_mode")
async def handle_system_mode(mode):
    mission_state["system_mode"] = mode
    logger.warning(f"[MissionManager] SYSTEM MODE → {mode}")

    if mode == "RECOVERY":
        for drone_id in DRONES:
            halt_drone(drone_id)

            # TODO: send both drones to launch point

def haversine_distance(lat1, lon1, lat2, lon2):
    # Earth radius in meters
    R = 6371000  

    # Convert degrees to radians
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    # Haversine formula
    a = math.sin(dphi / 2)**2 + \
        math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2)**2

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c

async def heartbeat_loop():
    while not shutdown_event.is_set():
        await redis.heartbeat()
        await asyncio.sleep(1)

async def main():
    global mav

    await redis.connect()

    logger.info("[MissionManager] Redis connected")

    mav = MAVLinkManager(
        redis,
        scout_uri=os.getenv("SCOUT_MAVLINK_UDP", "udp:localhost:13550"),
        sprayer_uri=os.getenv("SPRAYER_MAVLINK_UDP", "udp:localhost:13560"),
        loop=loop
    )

    tasks = [
        loop.create_task(heartbeat_loop()),
        # loop.create_task(mav.listen()),
        loop.create_task(mav.poll(mav.scout, "scout")),
        loop.create_task(mav.poll(mav.sprayer, "sprayer")),
    ]

    await shutdown_event.wait()

    for task in tasks:
        task.cancel()

    await redis.close()

def shutdown():
    logger.warning("[MissionManager] Shutdown initiated")
    shutdown_event.set()


def runner():
    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()


if __name__ == "__main__":
    runner()