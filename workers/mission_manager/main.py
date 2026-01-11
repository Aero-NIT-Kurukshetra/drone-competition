import asyncio
import json
import logging
import math
import os
import time
from typing import Dict, Any
from dotenv import load_dotenv

load_dotenv()

from common.redis_client import RedisClient
from workers.mission_manager.mavlink_manager import MAVLinkManager

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

logger = logging.getLogger("MissionManager")

WORKER_ID = "mission_manager"

loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

shutdown_event = asyncio.Event()
redis = RedisClient(loop=loop, worker_id=WORKER_ID)


mission_state: Dict[str, Any] = {
    "system_mode": "NORMAL",
    "drone_pose": {
        "scout": None,
        "sprayer": None,
    },
    "active_waypoint": {
        "scout": None,
        "sprayer": None,
    },
    "halted": {
        "scout": False,
        "sprayer": False,
    },
    "mode": {
        "scout": "IDLE",
        "sprayer": "IDLE",  # IDLE, NAVIGATING, SPRAYING
    },
    "current_target": {
        "scout": None,  #
        "sprayer": None,
    }
}

mav: MAVLinkManager | None = None


@redis.listen("start_mission")
async def handle_start_mission(_):
    logger.info("[MissionManager] Starting mission")

    for drone_id in ["scout", "sprayer"]:
        mission_state["halted"][drone_id] = False

    mav.set_guided_mode()
    await asyncio.sleep(2)

    mav.arm_and_takeoff("scout", 5) # altitude 5m

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

    if mission_state["system_mode"] == "RECOVERY":
        logger.warning(f"[MissionManager] Ignoring waypoint for {drone_id} (RECOVERY)")
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

    mission_state["active_waypoint"][drone_id] = waypoint
    mission_state["halted"][drone_id] = False

@redis.listen("event:no_safe_path")
async def handle_no_safe_path(data):
    drone_id = data["drone_id"]

    logger.warning(f"[MissionManager] NO SAFE PATH for {drone_id} → halting")

    mav.halt(drone_id)
    mission_state["halted"][drone_id] = True

@redis.listen("lidar_processing:occupancy_grid_updated")
async def handle_grid_update(_):  #recheck this function a
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
            "lat": crop["lat"],
            "lon": crop["lon"],
        }
    )


@redis.listen("sprayer:state_update")
async def handle_sprayer_state(data):
    """Listen for sprayer drone state changes."""
    state = data.get("state", "IDLE")
    previous_state = mission_state["mode"]["sprayer"]
    
    mission_state["mode"]["sprayer"] = state
    
    logger.info(f"[MissionManager] Sprayer state: {previous_state} → {state}")
    
    # If sprayer just became IDLE, check for next crop to spray
    if state == "IDLE" and previous_state != "IDLE":
        await dispatch_sprayer_to_next_crop()


async def complete_spraying_cycle():
    """
    Handle the spraying cycle:
    1. Lower altitude (if needed)
    2. Spray for preset duration
    3. Raise altitude
    4. Set state to IDLE → triggers next crop dispatch
    """
    SPRAY_DURATION_SECONDS = 5  # Preset spray time
    
    logger.info(f"[MissionManager] Spraying for {SPRAY_DURATION_SECONDS} seconds...")
    
    # TODO: Send MAVLink command to lower altitude
    # mav.set_altitude("sprayer", 2.0)
    
    # TODO: Send MAVLink START_SPRAYING command
    # mav.start_spraying("sprayer")
    
    await asyncio.sleep(SPRAY_DURATION_SECONDS)
    
    # TODO: Raise altitude back to safe height
    # mav.set_altitude("sprayer", 5.0)
    
    logger.info("[MissionManager] Spraying complete → setting IDLE")
    
    # Clear current target
    mission_state["current_target"]["sprayer"] = None
    mission_state["mode"]["sprayer"] = "IDLE"
    
    # Dispatch to next crop in queue
    await dispatch_sprayer_to_next_crop()


@redis.listen("mission_manager:drone_pose_update")
async def handle_pose_update(data):
    drone_id = data["drone_id"]
    mission_state["drone_pose"][drone_id] = data

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

    if drone_id == "scout" and not scout_waypoints:
        return
    
    if drone_id == "sprayer" and not sprayer_waypoints:
        return
    
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

    if drone_id == "scout" and distance < 1.0:  # 1.0m radius
        logger.info(f"[MissionManager] {drone_id} reached waypoint")

        mission_state["active_waypoint"][drone_id] = None

        await redis.publish(
            "mission_manager:request_next_waypoint",
            {"drone_id": drone_id, "lat": cur_lat, "lon": cur_lon}
        )

    elif drone_id == "sprayer":
        target = mission_state["current_target"]["sprayer"]
        
        if not target:
            return
            
        distance_to_target = haversine_distance(
            cur_lat, cur_lon, 
            target["lat"], target["lon"]
        )
        
        logger.info(f"[MissionManager] Sprayer distance to target: {distance_to_target:.2f}m")
        
        # Sprayer reached crop location
        if distance_to_target < 1.5 and mission_state["mode"]["sprayer"] == "NAVIGATING":
            logger.info("[MissionManager] Sprayer reached crop location → start spraying")
            
            mission_state["mode"]["sprayer"] = "SPRAYING"
            mission_state["active_waypoint"]["sprayer"] = None
            
            # TODO: Lower altitude, trigger spray mechanism via MAVLink
            # mav.start_spraying("sprayer")
            
            # For now, simulate spraying completion after a delay
            asyncio.create_task(complete_spraying_cycle()) 

@redis.listen("system_mode")
async def handle_system_mode(data):
    mode = data
    mission_state["system_mode"] = mode

    logger.warning(f"[MissionManager] SYSTEM MODE → {mode}")

    if mode == "RECOVERY":
        for drone_id in ["scout", "sprayer"]:
            mav.halt(drone_id)
            mission_state["halted"][drone_id] = True

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
