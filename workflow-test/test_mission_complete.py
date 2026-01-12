"""
Test 10: Mission Completion & RTL Test
Tests that both drones correctly return to launch when mission is complete.

Workflow tested:
1. Scout completes all waypoints → RTL
2. Sprayer finishes all crops → RTL
3. Both drones land at home position

This test simulates mission completion scenarios to verify RTL behavior.
"""
import asyncio
import json
import sys
import os
import time
import numpy as np

from dotenv import load_dotenv

load_dotenv()

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.redis_client import RedisClient
from workers.SETTINGS import (
    PATH_PLANNING_GRID_SIZE,
    LAT0, LON0,
    DEFAULT_ALTITUDE,
    GRID_FREE,
)


async def get_drone_modes(redis):
    """Get current modes for both drones."""
    state_raw = await redis.client.get("mission:state")
    if state_raw:
        state = json.loads(state_raw)
        scout = state["drones"]["scout"]["mode"]
        sprayer = state["drones"]["sprayer"]["mode"]
        return scout, sprayer
    return None, None


async def simulate_pose(redis, drone_id: str, lat: float, lon: float, alt: float):
    """Simulate drone pose update."""
    await redis.publish("mission_manager:drone_pose_update", {
        "drone_id": drone_id,
        "lat": lat,
        "lon": lon,
        "alt_m": alt,
        "timestamp": time.time()
    })


async def simulate_landed_state(redis, drone_id: str, landed_state: int):
    """Simulate extended sys state (landed state)."""
    await redis.publish("mission_manager:drone_extended_sys_state", {
        "drone_id": drone_id,
        "landed_state": landed_state,
        "timestamp": time.time()
    })


async def main():
    loop = asyncio.get_event_loop()
    redis = RedisClient(loop=loop, worker_id="test_rtl")
    
    await redis.connect()
    print("✓ Connected to Redis")
    
    print("\n" + "=" * 70)
    print("MISSION COMPLETION & RTL TEST")
    print("=" * 70)
    
    print("""
This test verifies RTL behavior:

  Test A: Scout RTL when waypoints complete
  Test B: Sprayer RTL when all crops sprayed
  Test C: Sprayer RTL when scout lands (no more exploration)

Prerequisites:
  - Mission Manager running
  - Path Planner running
""")
    
    # Setup
    print("[Setup] Initializing...")
    
    # Clear previous state
    keys_to_clear = [
        "crop_locations",
        "path_planner:current_crop_target_index",
        "path_planner:sprayer_waypoints",
        "path_planner:current_sprayer_waypoint_index",
    ]
    for key in keys_to_clear:
        await redis.client.delete(key)
    
    # Create fully explored grid
    grid = np.zeros((PATH_PLANNING_GRID_SIZE, PATH_PLANNING_GRID_SIZE), dtype=np.uint8)
    await redis.binary_client.set("occupancy_grid", grid.tobytes())
    
    # Create minimal scout waypoints (just 2 points)
    scout_waypoints = {
        "worker_id": "test",
        "waypoints": [
            {"lat": LAT0 - 0.0001, "lon": LON0, "alt_m": DEFAULT_ALTITUDE},
            {"lat": LAT0, "lon": LON0, "alt_m": DEFAULT_ALTITUDE},
        ],
        "timestamp": time.time()
    }
    await redis.client.set("path_planner:scout_waypoints", json.dumps(scout_waypoints))
    await redis.client.set("path_planner:current_scout_waypoint_index", "0")
    
    # Initialize drone positions
    await simulate_pose(redis, "scout", LAT0 - 0.0002, LON0, DEFAULT_ALTITUDE)
    await simulate_pose(redis, "sprayer", LAT0, LON0 - 0.0002, DEFAULT_ALTITUDE)
    
    print("✓ Setup complete\n")
    
    input("Press ENTER to start test (ensure Mission Manager is running)...")
    
    # ═══════════════════════════════════════════════════════════════════
    # TEST A: Scout RTL when waypoints complete
    # ═══════════════════════════════════════════════════════════════════
    print("\n" + "=" * 70)
    print("TEST A: Scout RTL on Waypoint Completion")
    print("=" * 70)
    
    # Start mission
    print("\n[1] Starting mission...")
    await redis.publish("start_mission", {})
    await asyncio.sleep(3)
    
    scout_mode, sprayer_mode = await get_drone_modes(redis)
    print(f"  Scout mode: {scout_mode}")
    print(f"  Sprayer mode: {sprayer_mode}")
    
    # Simulate scout at first waypoint
    print("\n[2] Simulating scout at waypoint 1...")
    wp1 = scout_waypoints["waypoints"][0]
    await simulate_pose(redis, "scout", wp1["lat"], wp1["lon"], DEFAULT_ALTITUDE)
    await asyncio.sleep(2)
    
    # Simulate scout at last waypoint
    print("\n[3] Simulating scout at final waypoint...")
    wp2 = scout_waypoints["waypoints"][1]
    await simulate_pose(redis, "scout", wp2["lat"], wp2["lon"], DEFAULT_ALTITUDE)
    await asyncio.sleep(2)
    
    # Scout should now be in RETURNING_TO_LAUNCH mode
    scout_mode, sprayer_mode = await get_drone_modes(redis)
    print(f"\n[4] Checking scout RTL...")
    print(f"  Scout mode: {scout_mode}")
    
    if scout_mode == "RETURNING_TO_LAUNCH":
        print("  ✓ SUCCESS: Scout entered RTL mode after completing waypoints")
    else:
        print(f"  ⚠ Expected RETURNING_TO_LAUNCH, got: {scout_mode}")
    
    # ═══════════════════════════════════════════════════════════════════
    # TEST B: Sprayer RTL when all crops sprayed
    # ═══════════════════════════════════════════════════════════════════
    print("\n" + "=" * 70)
    print("TEST B: Sprayer RTL on Crop Completion")
    print("=" * 70)
    
    # Add a single crop
    print("\n[1] Adding one crop to spray...")
    crop = {"lat": LAT0 + 0.0001, "lon": LON0}
    await redis.client.set("crop_locations", json.dumps([crop]))
    await redis.client.set("path_planner:current_crop_target_index", "-1")
    
    # Trigger sprayer planning
    await redis.publish("event:crop_detected", {
        "lat": crop["lat"],
        "lon": crop["lon"],
        "confidence": 0.9
    })
    await asyncio.sleep(2)
    
    # Simulate sprayer completing the crop (already at crop, in IDLE mode)
    # This simulates the state after all the spray states completed
    
    print("\n[2] Simulating sprayer reaching crop...")
    await simulate_pose(redis, "sprayer", crop["lat"], crop["lon"], DEFAULT_ALTITUDE)
    
    # Set crop index to indicate all processed
    await redis.client.set("path_planner:current_crop_target_index", "0")
    
    # Since scout is already in RTL, request next waypoint should trigger sprayer RTL
    print("\n[3] Triggering waypoint completion check...")
    await redis.publish("path_planning:planned_waypoint", {
        "drone_id": "sprayer",
        "waypoint": None  # No more waypoints
    })
    await asyncio.sleep(2)
    
    scout_mode, sprayer_mode = await get_drone_modes(redis)
    print(f"\n[4] Checking sprayer RTL...")
    print(f"  Scout mode: {scout_mode}")
    print(f"  Sprayer mode: {sprayer_mode}")
    
    if sprayer_mode == "RETURNING_TO_LAUNCH":
        print("  ✓ SUCCESS: Sprayer entered RTL mode after completing crops")
    else:
        print(f"  ⚠ Sprayer mode is: {sprayer_mode}")
        print("    (May enter RTL when scout lands)")
    
    # ═══════════════════════════════════════════════════════════════════
    # TEST C: Sprayer RTL when Scout Lands
    # ═══════════════════════════════════════════════════════════════════
    print("\n" + "=" * 70)
    print("TEST C: Sprayer RTL when Scout Lands")
    print("=" * 70)
    
    # Reset sprayer to IDLE to test this scenario
    if sprayer_mode != "RETURNING_TO_LAUNCH":
        print("\n[1] Simulating scout landing...")
        
        # Simulate scout has landed (landed_state = 1 = ON_GROUND)
        await simulate_landed_state(redis, "scout", 1)
        await asyncio.sleep(2)
        
        scout_mode, sprayer_mode = await get_drone_modes(redis)
        print(f"\n[2] After scout landing:")
        print(f"  Scout mode: {scout_mode}")
        print(f"  Sprayer mode: {sprayer_mode}")
        
        if sprayer_mode == "RETURNING_TO_LAUNCH":
            print("  ✓ SUCCESS: Sprayer entered RTL when scout landed")
        else:
            print(f"  ⚠ Sprayer did not RTL, mode is: {sprayer_mode}")
    else:
        print("\n  (Sprayer already in RTL from Test B)")
    
    # ═══════════════════════════════════════════════════════════════════
    # SUMMARY
    # ═══════════════════════════════════════════════════════════════════
    print("\n" + "=" * 70)
    print("RTL TEST SUMMARY")
    print("=" * 70)
    
    scout_mode, sprayer_mode = await get_drone_modes(redis)
    print(f"\n  Final Scout mode: {scout_mode}")
    print(f"  Final Sprayer mode: {sprayer_mode}")
    
    if scout_mode == "RETURNING_TO_LAUNCH" and sprayer_mode == "RETURNING_TO_LAUNCH":
        print("\n  ✓ PASS: Both drones correctly entered RTL mode")
    else:
        print("\n  ⚠ Check: Not all drones in RTL mode")
        print("    This may be expected if mission is still in progress")
    
    print("\n" + "=" * 70)
    
    await redis.close()


if __name__ == "__main__":
    asyncio.run(main())
