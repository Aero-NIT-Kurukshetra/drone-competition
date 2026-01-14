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
    SPRAYER_CRUISE_ALT,
    SPRAYER_SPRAY_ALT,
    SPRAYER_WAYPOINT_RADIUS,
    SPRAYER_CROP_RADIUS,
    SPRAYER_HOVER_TIME,
    SPRAYER_SPRAY_DURATION,
    LAT0,
    LON0,
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
        "last_pose_ts": None,      # timestamp of last pose update
        "landed_state": 0,         # 0=UNDEFINED, 1=ON_GROUND, 2=IN_AIR, 3=TAKEOFF, 4=LANDING
        "hover_start_ts": None,    # timestamp when hover started (for center correction)
        "spray_start_ts": None,    # timestamp when spraying started
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

    # Set guided mode for all drones
    mav.set_guided_mode()
    await asyncio.sleep(1)

    # Arm and takeoff scout
    await mav.arm_and_takeoff("scout", DEFAULT_ALTITUDE)
    set_mode("scout", "TAKING_OFF")
    
    # Wait for takeoff to complete before requesting waypoints
    await asyncio.sleep(5)

    await redis.publish(
        "mission_manager:request_next_waypoint",
        {"drone_id": "scout"}
    )

@redis.listen("path_planning:arm_takeoff")
async def handle_arm_takeoff(data):
    drone_id = data["drone_id"]
    altitude = data.get("altitude", DEFAULT_ALTITUDE)

    logger.info(f"[MissionManager] Arming and takeoff for {drone_id} to {altitude}m")

    await mav.arm_and_takeoff(drone_id, altitude)
    set_mode(drone_id, "TAKING_OFF")

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
        logger.info(f"[MissionManager] No waypoints for {drone_id}")
        mav.halt(drone_id)
        
        if drone_id == "scout":
            logger.info("[MissionManager] Scout mission complete → returning to launch")
            set_mode(drone_id, "RETURNING_TO_LAUNCH")
            await mav.return_to_launch(drone_id)
        else:
            # Sprayer: No path available - just go IDLE
            # Do NOT recursively call dispatch here
            set_mode(drone_id, "IDLE")
            logger.info("[MissionManager] Sprayer has no waypoints → IDLE")
        return

    logger.info(f"[MissionManager] Executing waypoint for {drone_id}: {waypoint}")

    # For sprayer: check if drone needs takeoff
    if drone_id == "sprayer":
        landed_state = drone.get("landed_state", 0)
        needs_takeoff = landed_state in (0, 1) or drone["mode"] in ("IDLE", "PLANNING", "WAITING_FOR_EXPLORATION")
        
        if needs_takeoff:
            logger.info(f"[MissionManager] Sprayer needs takeoff (landed_state={landed_state})")
            await mav.arm_and_takeoff("sprayer", DEFAULT_ALTITUDE)
            set_mode("sprayer", "TAKING_OFF")
            await asyncio.sleep(5)

    mav.send_waypoint_latlon(
        drone_id,
        waypoint["lat"],
        waypoint["lon"],
        waypoint.get("alt_m", DEFAULT_ALTITUDE)
    )

    set_active_waypoint(drone_id, waypoint)


@redis.listen("mission_manager:drone_extended_sys_state")
async def handle_extended_sys_state(data):
    """Track landed state for each drone.
    
    landed_state:
        0: UNDEFINED
        1: ON_GROUND
        2: IN_AIR
        3: TAKEOFF
        4: LANDING
    """
    drone_id = data["drone_id"]
    landed_state = data["landed_state"]
    mission_state["drones"][drone_id]["landed_state"] = landed_state
    
    # Check if scout has landed after RTL - trigger sprayer RTL if waiting
    if drone_id == "scout" and landed_state == 1:  # ON_GROUND
        scout_mode = mission_state["drones"]["scout"]["mode"]
        sprayer_mode = mission_state["drones"]["sprayer"]["mode"]
        
        if scout_mode == "RETURNING_TO_LAUNCH" and sprayer_mode in ("IDLE", "HOVERING", "WAITING_FOR_EXPLORATION"):
            # Scout has landed, sprayer should also return
            logger.info("[MissionManager] Scout landed after RTL → commanding sprayer to RTL")
            set_mode("sprayer", "RETURNING_TO_LAUNCH")
            await mav.return_to_launch("sprayer")


@redis.listen("mission_manager:drone_sprayer_finished")
async def handle_sprayer_finished(data):
    """Handle SPRAYER_FINISHED MAVLink message from sprayer Raspberry Pi.
    
    This is sent by sprayer_cam.py after completing:
    1. Centering over crop
    2. Descending to spray altitude
    3. Triggering spray relay
    4. Ascending back to cruise altitude
    
    Payload:
    {
        "drone_id": str,
        "result": int,
        "spray_time_ms": int,
        "timestamp": float
    }
    """
    drone_id = data["drone_id"]
    spray_time_ms = data.get("spray_time_ms", 0)
    result = data.get("result", 0)
    
    logger.info(f"[MissionManager] Sprayer Pi finished spray procedure (result={result}, duration={spray_time_ms}ms)")
    
    drone = mission_state["drones"][drone_id]
    
    if drone["mode"] == "WAITING_FOR_SPRAY_COMPLETION":
        # Pi has completed the entire spray procedure (center, descend, spray, ascend)
        logger.info("[MissionManager] Spray procedure complete → processing next crop")
        
        clear_active_waypoint("sprayer")
        
        # Handle crop completion (checks for more crops in queue)
        await on_sprayer_crop_completed()
    else:
        logger.warning(f"[MissionManager] Received SPRAYER_FINISHED but mode is {drone['mode']}, ignoring")

@redis.listen("event:no_safe_path")
async def handle_no_safe_path(data):
    drone_id = data["drone_id"]

    logger.warning(f"[MissionManager] NO SAFE PATH for {drone_id} → halting")

    halt_drone(drone_id)

@redis.listen("lidar_processing:occupancy_grid_updated")
async def handle_grid_update(_):
    # Any grid update invalidates current assumptions
    for drone_id in ["scout", "sprayer"]:
        if not mission_state["drones"][drone_id]["halted"]:
            await redis.publish(
                "mission_manager:request_next_waypoint",
                {"drone_id": drone_id}
            )

@redis.listen("event:crop_detected")
async def handle_crop_detected(data):
    """Handle crop detection from camera worker.
    
    This handler ONLY:
    1. Appends crop to crop_locations array
    2. Triggers sprayer dispatch IF sprayer is IDLE
    
    Payload:
    {
        "lat": float,
        "lon": float,
        "confidence": float (optional)
    }
    """
    crop_location = {
        "lat": data["lat"],
        "lon": data["lon"],
    }

    # Fetch current crop_locations array
    crop_locations_json = await redis.client.get("crop_locations")
    
    if crop_locations_json:
        crop_locations = json.loads(crop_locations_json)
    else:
        crop_locations = []
    
    # Append new crop location
    crop_locations.append(crop_location)
    await redis.client.set("crop_locations", json.dumps(crop_locations))

    logger.info(
        f"[MissionManager] Crop detected and queued → "
        f"lat={crop_location['lat']:.6f}, lon={crop_location['lon']:.6f} "
        f"(queue size: {len(crop_locations)})"
    )

    # Request path planner to mark crop location as explored
    await redis.publish(
        "path_planning:mark_explored",
        {
            "lat": crop_location["lat"],
            "lon": crop_location["lon"],
            "radius_m": 15.0
        }
    )

    # ONLY dispatch if sprayer is truly IDLE - no other conditions
    sprayer_mode = mission_state["drones"]["sprayer"]["mode"]
    if sprayer_mode == "IDLE":
        logger.info("[MissionManager] Sprayer is IDLE → dispatching to crop")
        await trigger_sprayer_planning()
    else:
        logger.info(f"[MissionManager] Sprayer busy ({sprayer_mode}) → crop queued for later")


@redis.listen("mission_manager:drone_crop_detected")
async def handle_mavlink_crop_detected(data):
    """Handle crop detection from MAVLink (drone onboard camera)."""
    logger.info(f"[MissionManager] MAVLink crop detected from {data['drone_id']}")
    
    await handle_crop_detected({
        "lat": data["lat"],
        "lon": data["lon"],
        "confidence": data.get("confidence", 100) / 100.0
    })


async def trigger_sprayer_planning():
    """
    Request path planning for sprayer to the next unprocessed crop.
    Uses the sprayer's CURRENT position for A* planning (not launch position).
    This is a simple trigger - all index management is in path_planner.
    """
    sprayer = mission_state["drones"]["sprayer"]
    
    if sprayer["halted"]:
        logger.warning("[MissionManager] Sprayer is halted, cannot dispatch")
        return
    
    # Get sprayer's current location from pose updates
    # CRITICAL: Use actual current position for A* to work correctly
    current_lat = sprayer["current_loc"].get("lat")
    current_lon = sprayer["current_loc"].get("lon")
    current_alt = sprayer["current_loc"].get("alt", DEFAULT_ALTITUDE)
    
    if current_lat is None or current_lon is None:
        # Fallback to home only if never received pose update
        logger.warning("[MissionManager] Sprayer position unknown, using home position")
        sprayer_pose = {"lat": LAT0, "lon": LON0, "alt": DEFAULT_ALTITUDE}
    else:
        sprayer_pose = {
            "lat": current_lat,
            "lon": current_lon,
            "alt": current_alt
        }
    
    logger.info(f"[MissionManager] Triggering sprayer A* planning from current position: lat={sprayer_pose['lat']:.6f}, lon={sprayer_pose['lon']:.6f}, alt={sprayer_pose['alt']:.1f}m")
    set_mode("sprayer", "PLANNING")
    
    await redis.publish(
        "mission_manager:sprayer_plan_request",
        {"drone_id": "sprayer", "drone_pose": sprayer_pose}
    )


async def on_sprayer_crop_completed():
    """
    Called when sprayer finishes spraying a crop.
    Checks for more crops in queue and dispatches if available.
    
    NOTE: We do NOT increment crop index here - path_planner handles that
    when handle_sprayer_planning_request is called.
    """
    # Get current index (already set by path_planner)
    current_index_raw = await redis.client.get("path_planner:current_crop_target_index")
    current_index = int(current_index_raw) if current_index_raw else 0
    
    logger.info(f"[MissionManager] Crop {current_index + 1} completed")
    
    # Check if there are more crops in queue
    crop_locations_json = await redis.client.get("crop_locations")
    crop_locations = json.loads(crop_locations_json) if crop_locations_json else []
    
    # Check if there are more crops to process (index is 0-based, so next would be current_index + 1)
    next_index = current_index + 1
    
    if next_index < len(crop_locations):
        # More crops available - dispatch to next from current position
        logger.info(f"[MissionManager] {len(crop_locations) - next_index} more crops in queue → dispatching from current position")
        await trigger_sprayer_planning()
    else:
        # No more crops currently - go IDLE and wait for new detections
        # Check if scout is done
        scout_mode = mission_state["drones"]["scout"]["mode"]
        if scout_mode == "RETURNING_TO_LAUNCH":
            logger.info("[MissionManager] All crops done, scout returned → sprayer RTL")
            set_mode("sprayer", "RETURNING_TO_LAUNCH")
            await mav.return_to_launch("sprayer")
        else:
            logger.info("[MissionManager] No more crops in queue, scout still flying → sprayer IDLE (will resume when new crop detected)")
            set_mode("sprayer", "IDLE")


# Track if sprayer is waiting for exploration
sprayer_waiting_for_exploration = False


@redis.listen("event:sprayer_waiting")
async def handle_sprayer_waiting(data):
    """Handle sprayer waiting for scout to explore the path."""
    global sprayer_waiting_for_exploration
    
    reason = data.get("reason", "unknown")
    crop_index = data.get("crop_index", -1)
    
    logger.info(f"[MissionManager] Sprayer waiting: {reason} (crop {crop_index + 1})")
    
    sprayer_waiting_for_exploration = True
    set_mode("sprayer", "WAITING_FOR_EXPLORATION")


# Counter to throttle exploration retry attempts
_exploration_retry_counter = 0
_EXPLORATION_RETRY_INTERVAL = 10  # Retry every N pose updates from scout


@redis.listen("mission_manager:drone_pose_update")
async def handle_pose_update(data):
    global sprayer_waiting_for_exploration, _exploration_retry_counter
    
    drone_id = data["drone_id"]
    drone = mission_state["drones"][drone_id]

    drone["last_pose_ts"] = data["timestamp"]
    
    # Update current location for all drones
    drone["current_loc"]["lat"] = data.get("lat")
    drone["current_loc"]["lon"] = data.get("lon")
    drone["current_loc"]["alt"] = data.get("alt_m", DEFAULT_ALTITUDE)
    
    # If scout moved and sprayer is waiting, retry planning periodically
    if drone_id == "scout" and sprayer_waiting_for_exploration:
        _exploration_retry_counter += 1
        if _exploration_retry_counter >= _EXPLORATION_RETRY_INTERVAL:
            _exploration_retry_counter = 0
            logger.info("[MissionManager] Scout explored more area, retrying sprayer planning...")
            sprayer_waiting_for_exploration = False
            await trigger_sprayer_planning()

    # Valid modes for waypoint tracking
    ACTIVE_MODES = [
        "NAVIGATING_TO_WAYPOINT", 
        "WAITING_FOR_NEXT_WAYPOINT", 
        "TAKING_OFF",
        "WAITING_FOR_SPRAY_COMPLETION",  # Pi handles spray procedure
    ]

    if drone["halted"] or drone["mode"] not in ACTIVE_MODES:
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

        is_last_waypoint = sprayer_current_wp_index == len(sprayer_waypoints) - 1

        # ──────────────────────────────
        # 1. Normal waypoint navigation
        # ──────────────────────────────
        if not is_last_waypoint:
            if distance < SPRAYER_WAYPOINT_RADIUS:
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

        # Log current state for debugging (only on last waypoint)
        logger.debug(f"[MissionManager] sprayer spray-state-check: mode={drone['mode']}, dist={distance:.2f}m, alt={cur_alt:.1f}m, is_last_wp={is_last_waypoint}")

        # ──────────────────────────────
        # 2. Crop waypoint reached → send spray command to Raspberry Pi
        # ──────────────────────────────
        if drone["mode"] == "NAVIGATING_TO_WAYPOINT" and distance < SPRAYER_CROP_RADIUS:
            logger.info(f"[MissionManager] sprayer reached crop (dist={distance:.2f}m < {SPRAYER_CROP_RADIUS}m)")
            logger.info("[MissionManager] Sending SPRAY_COMMAND to sprayer Raspberry Pi...")

            # Halt drone - Pi will take over positioning
            mav.halt("sprayer")
            
            # Send spray command to Pi via STATUSTEXT
            # Pi will handle: centering, descending, spraying, ascending
            mav.send_spray_command(wp["lat"], wp["lon"])
            
            # Set mode to wait for Pi to complete spray procedure
            drone["mode"] = "WAITING_FOR_SPRAY_COMPLETION"
            await sync_state_to_redis()
            return

        # ──────────────────────────────
        # 3. Waiting for spray completion from Pi
        # ──────────────────────────────
        # The SPRAYER_FINISHED event from Pi is handled by handle_sprayer_finished()
        # which will trigger on_sprayer_crop_completed()

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
        loop.create_task(mav.poll()),
        # loop.create_task(mav.poll(mav.scout, "scout")),
        # loop.create_task(mav.poll(mav.sprayer, "sprayer")),
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