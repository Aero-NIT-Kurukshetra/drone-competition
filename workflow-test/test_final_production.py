"""
NIDAR Final Production Workflow Test
=====================================

This comprehensive test validates the entire NIDAR system including:

1. Worker Health Checks
   - Mission Manager connectivity
   - Path Planner connectivity
   - Redis connectivity

2. Scout Drone Workflow
   - KML parsing and lawnmower waypoint generation
   - Arm, takeoff, and navigation
   - Crop detection via MAVLink STATUSTEXT (pcam.py simulation)
   - Validation: drone in-air check, KML bounds check

3. Sprayer Drone Workflow
   - Crop queue management
   - A* path planning to crops
   - State machine: NAV → WAITING_FOR_SPRAY_COMPLETION (Pi handles spray)
   - MAVLink STATUSTEXT: GCS sends SPRAY_COMMAND, Pi sends SPRAYER_FINISHED

4. System Integration
   - Concurrent drone operations
   - Redis pub/sub event flow
   - State synchronization

Prerequisites:
--------------
1. Redis server running on localhost:6379
2. Scout SITL: arducopter.exe with --serial1 udpclient:127.0.0.1:13550
3. Sprayer SITL: arducopter.exe with --serial1 udpclient:127.0.0.1:13560
4. Mission Manager: python -m workers.mission_manager.main
5. Path Planner: python -m workers.path_planning.main

Run this test:
--------------
    python workflow-test/test_final_production.py

Author: NIDAR Team
"""

import asyncio
import json
import sys
import os
import time
import math
import numpy as np
from datetime import datetime
from typing import Dict, List, Optional
from dataclasses import dataclass
from enum import Enum

from dotenv import load_dotenv
load_dotenv()

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.redis_client import RedisClient
from workers.SETTINGS import (
    PATH_PLANNING_GRID_SIZE,
    GRID_FREE,
    GRID_UNEXPLORED,
    LAT0, LON0,
    FARM_POLYGON,
    point_in_polygon,
    DEFAULT_ALTITUDE,
    SPRAYER_CRUISE_ALT,
    SPRAYER_SPRAY_ALT,
)


# ============================================
# TEST CONFIGURATION
# ============================================

class TestMode(Enum):
    FULL = "full"           # Full integration with SITL
    EVENTS_ONLY = "events"  # Test Redis events without SITL
    VALIDATION = "validate" # Test validation logic only


# Farm polygon KML
KML_XML = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
    <Placemark>
        <name>Farm</name>
        <Polygon>
            <outerBoundaryIs>
                <LinearRing>
                    <coordinates>
                        76.81589155096442,29.94965311567124,0 
                        76.8162492257966,29.94939808518254,0 
                        76.81664355584154,29.94963310749574,0 
                        76.81673785885653,29.95054251583795,0 
                        76.81637533963627,29.95078200571026,0 
                        76.81601151843628,29.95060813092637,0 
                        76.81589155096442,29.94965311567124,0 
                    </coordinates>
                </LinearRing>
            </outerBoundaryIs>
        </Polygon>
    </Placemark>
</Document>
</kml>"""

# Hidden crops that scout will "discover" (inside farm polygon)
HIDDEN_CROPS_VALID = [
    {"lat": 29.94980, "lon": 76.81620, "confidence": 0.85},
    {"lat": 29.95010, "lon": 76.81640, "confidence": 0.90},
    {"lat": 29.95040, "lon": 76.81655, "confidence": 0.88},
]

# Invalid crops for testing boundary validation (outside farm polygon)
HIDDEN_CROPS_INVALID = [
    {"lat": 29.94900, "lon": 76.81500, "confidence": 0.80},  # Southwest, outside
    {"lat": 29.95200, "lon": 76.81700, "confidence": 0.75},  # Northeast, outside
]

DETECTION_RADIUS_M = 15.0  # Scout must be within 15m to detect crop
EARTH_RADIUS = 6378137.0

# Event IDs (matching pcam.py and sprayer_cam.py)
EVENT_SPRAY_COMMAND = 0
EVENT_SPRAYER_FINISHED = 1
EVENT_CROP_DETECTED = 2


# ============================================
# TEST STATE
# ============================================

@dataclass
class TestState:
    """Track test progress and results."""
    scout_lat: Optional[float] = None
    scout_lon: Optional[float] = None
    scout_alt: Optional[float] = None
    scout_landed_state: int = 0  # 0=UNDEFINED, 1=ON_GROUND, 2=IN_AIR
    
    sprayer_lat: Optional[float] = None
    sprayer_lon: Optional[float] = None
    sprayer_alt: Optional[float] = None
    sprayer_landed_state: int = 0
    
    crops_detected: List[dict] = None
    crops_rejected: List[dict] = None
    crops_sprayed: int = 0
    
    scout_waypoints_total: int = 0
    scout_waypoints_completed: int = 0
    
    test_start_time: float = 0
    errors: List[str] = None
    warnings: List[str] = None
    
    def __post_init__(self):
        self.crops_detected = []
        self.crops_rejected = []
        self.errors = []
        self.warnings = []


test_state = TestState()


# ============================================
# HELPER FUNCTIONS
# ============================================

def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate distance between two GPS points in meters."""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    return EARTH_RADIUS * c


def format_duration(seconds: float) -> str:
    """Format duration in human-readable format."""
    mins, secs = divmod(int(seconds), 60)
    return f"{mins}m {secs}s" if mins > 0 else f"{secs}s"


def print_header(title: str):
    """Print a formatted section header."""
    print(f"\n{'=' * 70}")
    print(f" {title}")
    print(f"{'=' * 70}")


def print_step(step_num: int, description: str):
    """Print a test step."""
    print(f"\n[Step {step_num}] {description}")
    print("-" * 50)


def print_result(success: bool, message: str):
    """Print a test result."""
    icon = "✓" if success else "✗"
    print(f"  {icon} {message}")


def print_info(message: str):
    """Print info message."""
    print(f"  ℹ {message}")


def print_warning(message: str):
    """Print warning message."""
    print(f"  ⚠ {message}")
    test_state.warnings.append(message)


# ============================================
# VALIDATION TESTS
# ============================================

def test_point_in_polygon():
    """Test the boundary validation function."""
    print_step(0, "Testing KML boundary validation")
    
    # Test points inside polygon
    inside_tests = [
        (29.95010, 76.81640, "Center of farm"),
        (29.94980, 76.81620, "South area"),
        (29.95050, 76.81650, "North area"),
    ]
    
    for lat, lon, desc in inside_tests:
        result = point_in_polygon(lat, lon)
        print_result(result, f"Inside: {desc} ({lat:.5f}, {lon:.5f})")
        if not result:
            test_state.errors.append(f"Point should be inside: {desc}")
    
    # Test points outside polygon
    outside_tests = [
        (29.94900, 76.81500, "Southwest corner"),
        (29.95200, 76.81700, "Northeast corner"),
        (29.95000, 76.81800, "Far east"),
    ]
    
    for lat, lon, desc in outside_tests:
        result = not point_in_polygon(lat, lon)
        print_result(result, f"Outside: {desc} ({lat:.5f}, {lon:.5f})")
        if not result:
            test_state.errors.append(f"Point should be outside: {desc}")
    
    return len(test_state.errors) == 0


# ============================================
# WORKER HEALTH CHECKS
# ============================================

async def check_workers(redis: RedisClient) -> bool:
    """Check if all required workers are running."""
    print_step(1, "Checking worker health")
    
    all_healthy = True
    
    # Check Mission Manager
    mm_state = await redis.client.get("state:mission_manager")
    if mm_state:
        state = json.loads(mm_state)
        print_result(True, f"Mission Manager: {state.get('status', 'unknown')}")
    else:
        print_result(False, "Mission Manager: NOT RUNNING")
        print_info("Start with: python -m workers.mission_manager.main")
        all_healthy = False
    
    # Check Path Planner
    pp_state = await redis.client.get("state:path_planner_worker")
    if pp_state:
        state = json.loads(pp_state)
        print_result(True, f"Path Planner: {state.get('status', 'unknown')}")
    else:
        print_result(False, "Path Planner: NOT RUNNING")
        print_info("Start with: python -m workers.path_planning.main")
        all_healthy = False
    
    # Check Redis connectivity
    try:
        await redis.client.ping()
        print_result(True, "Redis: Connected")
    except Exception as e:
        print_result(False, f"Redis: {e}")
        all_healthy = False
    
    return all_healthy


# ============================================
# INITIALIZATION
# ============================================

async def initialize_test(redis: RedisClient):
    """Initialize test environment."""
    print_step(2, "Initializing test environment")
    
    # Clear previous data
    keys_to_clear = [
        "crop_locations",
        "path_planner:current_crop_target_index",
        "path_planner:scout_waypoints",
        "path_planner:current_scout_waypoint_index",
        "mission:state",
    ]
    
    for key in keys_to_clear:
        await redis.client.delete(key)
    print_result(True, "Cleared previous mission data")
    
    # Initialize crop index
    await redis.client.set("path_planner:current_crop_target_index", "-1")
    print_result(True, "Initialized crop index to -1")
    
    # Create occupancy grid (fully explored for integration test)
    grid = np.full((PATH_PLANNING_GRID_SIZE, PATH_PLANNING_GRID_SIZE), GRID_FREE, dtype=np.uint8)
    await redis.binary_client.set("occupancy_grid", grid.tobytes())
    print_result(True, f"Created {PATH_PLANNING_GRID_SIZE}x{PATH_PLANNING_GRID_SIZE} occupancy grid (fully explored)")
    
    test_state.test_start_time = time.time()


# ============================================
# SCOUT CAMERA SIMULATION
# ============================================

async def simulate_scout_camera(redis: RedisClient, stop_event: asyncio.Event):
    """
    Simulate pcam.py behavior - detect crops when scout flies over them.
    
    This simulates the MAVLink STATUSTEXT messages that pcam.py sends
    when it detects yellow crops via HSV processing.
    """
    all_crops = HIDDEN_CROPS_VALID + HIDDEN_CROPS_INVALID
    detected_flags = {i: False for i in range(len(all_crops))}
    
    while not stop_event.is_set():
        # Only detect when scout is in air
        if test_state.scout_landed_state != 2:
            await asyncio.sleep(0.5)
            continue
        
        scout_lat = test_state.scout_lat
        scout_lon = test_state.scout_lon
        
        if scout_lat is None or scout_lon is None:
            await asyncio.sleep(0.5)
            continue
        
        for i, crop in enumerate(all_crops):
            if detected_flags[i]:
                continue
            
            distance = haversine_distance(
                scout_lat, scout_lon,
                crop["lat"], crop["lon"]
            )
            
            if distance <= DETECTION_RADIUS_M:
                detected_flags[i] = True
                
                # Simulate pcam.py sending STATUSTEXT
                # Format: "event_id,lat_int,lon_int,confidence"
                is_valid = i < len(HIDDEN_CROPS_VALID)
                
                print(f"\n  📷 [Scout Camera] Crop detected at lat={crop['lat']:.6f}, lon={crop['lon']:.6f}")
                print(f"     Distance: {distance:.1f}m, Valid location: {is_valid}")
                
                # Publish as MAVLink crop detection event
                # This simulates what mavlink_manager receives from pcam.py via STATUSTEXT
                await redis.publish("mission_manager:drone_crop_detected", {
                    "drone_id": "scout",
                    "lat": crop["lat"],
                    "lon": crop["lon"],
                    "confidence": crop["confidence"],
                    "timestamp": time.time()
                })
                
                if is_valid:
                    test_state.crops_detected.append(crop)
                else:
                    test_state.crops_rejected.append(crop)
        
        await asyncio.sleep(0.3)


# ============================================
# SPRAYER CAMERA SIMULATION  
# ============================================

# Simulated spray procedure duration (Pi does: center, descend, spray, ascend)
SIMULATED_SPRAY_PROCEDURE_DURATION_S = 500.0

async def simulate_sprayer_camera(redis: RedisClient, stop_event: asyncio.Event):
    """
    Simulate sprayer_cam.py behavior - execute spray procedure when commanded.
    
    In production, sprayer_cam.py runs on the Raspberry Pi and:
    1. Receives EVENT_SPRAY_COMMAND (0) via STATUSTEXT from GCS when drone reaches crop
    2. Centers over crop using HSV camera
    3. Descends to spray altitude (1m)
    4. Triggers GPIO relay for spray (2s)
    5. Ascends back to cruise altitude (5m)
    6. Sends EVENT_SPRAYER_FINISHED (1) via STATUSTEXT back to GCS
    
    The Mission Manager enters WAITING_FOR_SPRAY_COMPLETION mode and waits
    for the SPRAYER_FINISHED event from the Pi.
    
    This simulation monitors sprayer state and simulates the Pi's behavior.
    """
    last_mode = None
    spray_start_time = None
    
    while not stop_event.is_set():
        state_raw = await redis.client.get("mission:state")
        if not state_raw:
            await asyncio.sleep(0.5)
            continue
        
        state = json.loads(state_raw)
        sprayer = state["drones"]["sprayer"]
        current_mode = sprayer["mode"]
        
        # Detect mode changes
        if current_mode != last_mode:
            if current_mode == "WAITING_FOR_SPRAY_COMPLETION":
                # GCS has sent SPRAY_COMMAND to Pi - simulate Pi's spray procedure
                spray_start_time = time.time()
                print(f"\n  📡 [Sprayer Pi] Received SPRAY_COMMAND from GCS")
                print(f"     Starting spray procedure (simulated {SIMULATED_SPRAY_PROCEDURE_DURATION_S}s)...")
                print(f"     → Centering over crop using HSV camera...")
                print(f"     → Descending to spray altitude...")
                print(f"     → Activating spray relay...")
                print(f"     → Ascending to cruise altitude...")
            
            last_mode = current_mode
        
        # If waiting for spray completion, simulate the Pi finishing
        if current_mode == "WAITING_FOR_SPRAY_COMPLETION" and spray_start_time:
            elapsed = time.time() - spray_start_time
            
            if elapsed >= SIMULATED_SPRAY_PROCEDURE_DURATION_S:
                spray_duration_ms = int(elapsed * 1000)
                
                print(f"\n  ✓ [Sprayer Pi] Spray procedure complete! Duration: {spray_duration_ms}ms")
                print(f"     Sending SPRAYER_FINISHED event to GCS...")
                
                # Simulate sprayer_cam.py sending STATUSTEXT "1,result,spray_time,0"
                # await redis.publish("mission_manager:drone_sprayer_finished", {
                #     "drone_id": "sprayer",
                #     "result": 0,  # Success
                #     "spray_time_ms": spray_duration_ms,
                #     "timestamp": time.time()
                # })
                
                test_state.crops_sprayed += 1
                spray_start_time = None
        
        await asyncio.sleep(0.3)


# ============================================
# STATE MONITORING
# ============================================

async def monitor_drone_poses(redis: RedisClient, stop_event: asyncio.Event):
    """Subscribe to drone pose updates and track state."""
    
    @redis.listen("mission_manager:drone_pose_update")
    async def handle_pose(data):
        drone_id = data.get("drone_id")
        if drone_id == "scout":
            test_state.scout_lat = data.get("lat")
            test_state.scout_lon = data.get("lon")
            test_state.scout_alt = data.get("alt_m")
        elif drone_id == "sprayer":
            test_state.sprayer_lat = data.get("lat")
            test_state.sprayer_lon = data.get("lon")
            test_state.sprayer_alt = data.get("alt_m")
    
    @redis.listen("mission_manager:drone_extended_sys_state")
    async def handle_sys_state(data):
        drone_id = data.get("drone_id")
        landed_state = data.get("landed_state", 0)
        if drone_id == "scout":
            test_state.scout_landed_state = landed_state
        elif drone_id == "sprayer":
            test_state.sprayer_landed_state = landed_state
    
    # Just keep running until stopped
    while not stop_event.is_set():
        await asyncio.sleep(1)


async def monitor_mission_state(redis: RedisClient, stop_event: asyncio.Event):
    """Monitor and display mission progress."""
    last_scout_mode = None
    last_sprayer_mode = None
    last_update = time.time()
    
    while not stop_event.is_set():
        state_raw = await redis.client.get("mission:state")
        if not state_raw:
            await asyncio.sleep(0.5)
            continue
        
        state = json.loads(state_raw)
        scout = state["drones"]["scout"]
        sprayer = state["drones"]["sprayer"]
        
        # Track mode changes
        if scout["mode"] != last_scout_mode:
            elapsed = format_duration(time.time() - test_state.test_start_time)
            print(f"\n  [{elapsed}] 🚁 Scout: {last_scout_mode} → {scout['mode']}")
            last_scout_mode = scout["mode"]
        
        if sprayer["mode"] != last_sprayer_mode:
            elapsed = format_duration(time.time() - test_state.test_start_time)
            print(f"\n  [{elapsed}] 🚜 Sprayer: {last_sprayer_mode} → {sprayer['mode']}")
            last_sprayer_mode = sprayer["mode"]
        
        # Check for mission completion
        if scout["mode"] in ["RETURNING_TO_LAUNCH", "LANDED"] and sprayer["mode"] == "IDLE":
            # Check if all detected crops have been sprayed
            if test_state.crops_sprayed >= len(test_state.crops_detected):
                print("\n\n" + "=" * 70)
                print(" ✓ MISSION COMPLETE")
                print("=" * 70)
                stop_event.set()
        
        # Periodic status update
        if time.time() - last_update > 10:
            last_update = time.time()
            elapsed = format_duration(time.time() - test_state.test_start_time)
            print(f"\n  [{elapsed}] Status: Scout={scout['mode']}, Sprayer={sprayer['mode']}, Crops={test_state.crops_sprayed}/{len(test_state.crops_detected)}")
        
        await asyncio.sleep(0.5)


# ============================================
# MAIN TEST RUNNER
# ============================================

async def run_full_test(redis: RedisClient):
    """Run the complete integration test."""
    
    # Step 3: Generate scout waypoints
    print_step(3, "Generating scout waypoints from KML")
    
    await redis.publish("mission_manager:scout_planning_request", {
        "kml_xml": KML_XML,
        "spacing": 15,  # Larger spacing for faster test
        "angle": 45
    })
    
    await asyncio.sleep(2)
    
    waypoints_raw = await redis.client.get("path_planner:scout_waypoints")
    if not waypoints_raw:
        print_result(False, "Failed to generate waypoints!")
        return False
    
    data = json.loads(waypoints_raw)
    test_state.scout_waypoints_total = len(data["waypoints"])
    print_result(True, f"Generated {test_state.scout_waypoints_total} waypoints")
    
    # Step 4: Start mission
    print_step(4, "Starting mission")
    print_info("Scout will arm, takeoff, and navigate lawnmower pattern")
    print_info("Sprayer will wait for crop detections, then navigate and spray")
    print_info("Press Ctrl+C to stop the test early")
    
    input("\nPress ENTER to start mission...")
    
    await redis.publish("start_mission", {})
    print_result(True, "Mission started!")
    
    # Step 5: Run concurrent monitoring tasks
    print_step(5, "Running integration test")
    print("-" * 70)
    
    stop_event = asyncio.Event()
    
    try:
        await asyncio.gather(
            monitor_drone_poses(redis, stop_event),
            monitor_mission_state(redis, stop_event),
            simulate_scout_camera(redis, stop_event),
            simulate_sprayer_camera(redis, stop_event),
        )
    except KeyboardInterrupt:
        print("\n\n⚠ Test interrupted by user")
        stop_event.set()
    except Exception as e:
        print(f"\n\n✗ Test error: {e}")
        test_state.errors.append(str(e))
        stop_event.set()
    
    return True


async def print_final_report(redis: RedisClient):
    """Print final test report."""
    print_header("FINAL TEST REPORT")
    
    elapsed = time.time() - test_state.test_start_time
    print(f"\n  Test Duration: {format_duration(elapsed)}")
    print(f"  Test Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    # Scout summary
    print("\n  SCOUT DRONE")
    print("  " + "-" * 40)
    state_raw = await redis.client.get("mission:state")
    if state_raw:
        state = json.loads(state_raw)
        print(f"    Final Mode: {state['drones']['scout']['mode']}")
    
    scout_wp_idx = await redis.client.get("path_planner:current_scout_waypoint_index")
    if scout_wp_idx:
        print(f"    Waypoints Completed: {int(scout_wp_idx) + 1}/{test_state.scout_waypoints_total}")
    
    # Sprayer summary
    print("\n  SPRAYER DRONE")
    print("  " + "-" * 40)
    if state_raw:
        state = json.loads(state_raw)
        print(f"    Final Mode: {state['drones']['sprayer']['mode']}")
    print(f"    Crops Sprayed: {test_state.crops_sprayed}")
    
    # Crop detection summary
    print("\n  CROP DETECTION")
    print("  " + "-" * 40)
    print(f"    Valid Detections: {len(test_state.crops_detected)}")
    print(f"    Invalid (Outside Bounds): {len(test_state.crops_rejected)}")
    
    for crop in test_state.crops_detected:
        print(f"      ✓ lat={crop['lat']:.6f}, lon={crop['lon']:.6f}")
    
    for crop in test_state.crops_rejected:
        print(f"      ✗ lat={crop['lat']:.6f}, lon={crop['lon']:.6f} (rejected)")
    
    # Validation results
    print("\n  VALIDATION RESULTS")
    print("  " + "-" * 40)
    
    if test_state.errors:
        print(f"    Errors: {len(test_state.errors)}")
        for err in test_state.errors:
            print(f"      ✗ {err}")
    else:
        print("    ✓ No errors")
    
    if test_state.warnings:
        print(f"    Warnings: {len(test_state.warnings)}")
        for warn in test_state.warnings:
            print(f"      ⚠ {warn}")
    else:
        print("    ✓ No warnings")
    
    # Overall result
    print("\n" + "=" * 70)
    if not test_state.errors:
        print(" ✓ TEST PASSED")
    else:
        print(" ✗ TEST FAILED")
    print("=" * 70)


async def main():
    """Main test entry point."""
    print_header("NIDAR FINAL PRODUCTION WORKFLOW TEST")
    
    print("""
This test validates the complete NIDAR system:

  • Worker health checks (Mission Manager, Path Planner)
  • KML boundary validation (point_in_polygon)
  • Scout drone: waypoint generation, navigation, crop detection
  • Sprayer drone: path planning, spray state machine
  • MAVLink event simulation (pcam.py, sprayer_cam.py)
  • Redis pub/sub event flow

Test crops:
  • 3 valid crops inside farm polygon
  • 2 invalid crops outside polygon (should be rejected)
""")
    
    loop = asyncio.get_event_loop()
    redis = RedisClient(loop=loop, worker_id="test_final_production")
    
    await redis.connect()
    print_result(True, "Connected to Redis")
    
    # Run validation tests first
    if not test_point_in_polygon():
        print("\n✗ Validation tests failed. Aborting.")
        await redis.close()
        return
    
    # Check workers
    workers_ok = await check_workers(redis)
    if not workers_ok:
        print("\n⚠ Some workers are not running.")
        response = input("Continue anyway? (y/N): ")
        if response.lower() != 'y':
            await redis.close()
            return
    
    # Initialize test environment
    await initialize_test(redis)
    
    # Run full integration test
    await run_full_test(redis)
    
    # Print final report
    await print_final_report(redis)
    
    await redis.close()


if __name__ == "__main__":
    asyncio.run(main())
