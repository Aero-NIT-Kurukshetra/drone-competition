"""
Test 9: Sprayer State Machine Test
Tests the complete sprayer spray sequence state machine:

1. NAVIGATING_TO_WAYPOINT → approaching crop
2. HOVERING_FOR_CORRECTION → center correction (1s)
3. DESCENDING_TO_SPRAY → descend to spray altitude
4. SPRAYING → spray duration (2s default)
5. ASCENDING_AFTER_SPRAY → return to cruise altitude
6. IDLE/PLANNING → dispatch to next crop

This test mocks the drone pose updates to simulate the sprayer
moving through each state transition.
"""
import asyncio
import json
import sys
import os
import time

from dotenv import load_dotenv

load_dotenv()

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.redis_client import RedisClient
from workers.SETTINGS import (
    PATH_PLANNING_GRID_SIZE,
    LAT0, LON0,
    SPRAYER_CRUISE_ALT,
    SPRAYER_SPRAY_ALT,
    SPRAYER_WAYPOINT_RADIUS,
    SPRAYER_CROP_RADIUS,
    SPRAYER_HOVER_TIME,
    SPRAYER_SPRAY_DURATION,
    GRID_FREE,
)
import numpy as np


# Test crop location (center of farm)
TEST_CROP = {"lat": LAT0, "lon": LON0}

# Sprayer start position (slightly south)
SPRAYER_START = {"lat": LAT0 - 0.0002, "lon": LON0}


async def wait_for_mode(redis, target_mode: str, timeout: float = 30.0):
    """Wait for sprayer to reach a specific mode."""
    start_time = time.time()
    while time.time() - start_time < timeout:
        state_raw = await redis.client.get("mission:state")
        if state_raw:
            state = json.loads(state_raw)
            current_mode = state["drones"]["sprayer"]["mode"]
            if current_mode == target_mode:
                return True
        await asyncio.sleep(0.2)
    return False


async def get_sprayer_mode(redis):
    """Get current sprayer mode."""
    state_raw = await redis.client.get("mission:state")
    if state_raw:
        state = json.loads(state_raw)
        return state["drones"]["sprayer"]["mode"]
    return None


async def simulate_pose(redis, lat: float, lon: float, alt: float):
    """Simulate sprayer pose update."""
    await redis.publish("mission_manager:drone_pose_update", {
        "drone_id": "sprayer",
        "lat": lat,
        "lon": lon,
        "alt_m": alt,
        "timestamp": time.time()
    })


async def main():
    loop = asyncio.get_event_loop()
    redis = RedisClient(loop=loop, worker_id="test_spray_states")
    
    await redis.connect()
    print("✓ Connected to Redis")
    
    print("\n" + "=" * 70)
    print("SPRAYER STATE MACHINE TEST")
    print("=" * 70)
    
    print(f"""
Configuration:
  Cruise altitude: {SPRAYER_CRUISE_ALT}m
  Spray altitude:  {SPRAYER_SPRAY_ALT}m
  Waypoint radius: {SPRAYER_WAYPOINT_RADIUS}m
  Crop radius:     {SPRAYER_CROP_RADIUS}m
  Hover time:      {SPRAYER_HOVER_TIME}s
  Spray duration:  {SPRAYER_SPRAY_DURATION}s
  
Test crop:     lat={TEST_CROP['lat']}, lon={TEST_CROP['lon']}
Sprayer start: lat={SPRAYER_START['lat']}, lon={SPRAYER_START['lon']}
""")
    
    # Setup: Clear and initialize
    print("[Setup] Initializing...")
    await redis.client.delete("crop_locations")
    await redis.client.delete("path_planner:current_crop_target_index")
    await redis.client.delete("path_planner:sprayer_waypoints")
    await redis.client.delete("path_planner:current_sprayer_waypoint_index")
    
    # Create fully explored grid (no obstacles)
    grid = np.zeros((PATH_PLANNING_GRID_SIZE, PATH_PLANNING_GRID_SIZE), dtype=np.uint8)
    await redis.binary_client.set("occupancy_grid", grid.tobytes())
    
    # Set initial sprayer position
    await simulate_pose(redis, SPRAYER_START["lat"], SPRAYER_START["lon"], SPRAYER_CRUISE_ALT)
    await asyncio.sleep(0.5)
    
    print("✓ Setup complete\n")
    
    input("Press ENTER to start (ensure Mission Manager + Path Planner running)...")
    
    # Start mission
    print("\n[1] Starting mission...")
    await redis.publish("start_mission", {})
    await asyncio.sleep(2)
    
    # Simulate a crop detection
    print("\n[2] Detecting crop to trigger sprayer...")
    await redis.publish("event:crop_detected", {
        "lat": TEST_CROP["lat"],
        "lon": TEST_CROP["lon"],
        "confidence": 0.9
    })
    await asyncio.sleep(2)
    
    # Wait for planning to complete
    sprayer_wp_raw = await redis.client.get("path_planner:sprayer_waypoints")
    if not sprayer_wp_raw:
        print("✗ No sprayer waypoints generated. Check path planner.")
        await redis.close()
        return
    
    wp_data = json.loads(sprayer_wp_raw)
    print(f"✓ Path planned with {len(wp_data['waypoints'])} waypoints")
    
    # Get the target waypoint (last one is the crop)
    target_wp = wp_data["waypoints"][-1]
    
    print("\n" + "-" * 70)
    print("STATE MACHINE TEST")
    print("-" * 70)
    
    # ═══════════════════════════════════════════════════════════════════
    # STATE 1: NAVIGATING_TO_WAYPOINT
    # ═══════════════════════════════════════════════════════════════════
    print("\n[State 1] NAVIGATING_TO_WAYPOINT")
    
    current_mode = await get_sprayer_mode(redis)
    print(f"  Current mode: {current_mode}")
    
    # Simulate sprayer approaching the crop (but not within crop radius yet)
    approach_lat = target_wp["lat"] - 0.00002  # ~2m away
    approach_lon = target_wp["lon"]
    
    print(f"  Simulating approach to crop ({approach_lat:.6f}, {approach_lon:.6f})...")
    await simulate_pose(redis, approach_lat, approach_lon, SPRAYER_CRUISE_ALT)
    await asyncio.sleep(1)
    
    current_mode = await get_sprayer_mode(redis)
    print(f"  Mode after approach: {current_mode}")
    
    # ═══════════════════════════════════════════════════════════════════
    # STATE 2: HOVERING_FOR_CORRECTION
    # ═══════════════════════════════════════════════════════════════════
    print("\n[State 2] HOVERING_FOR_CORRECTION")
    
    # Move to within crop radius
    print(f"  Simulating arrival at crop ({target_wp['lat']:.6f}, {target_wp['lon']:.6f})...")
    await simulate_pose(redis, target_wp["lat"], target_wp["lon"], SPRAYER_CRUISE_ALT)
    
    # Wait for state transition
    if await wait_for_mode(redis, "HOVERING_FOR_CORRECTION", timeout=5):
        print("  ✓ Entered HOVERING_FOR_CORRECTION")
    else:
        current_mode = await get_sprayer_mode(redis)
        print(f"  ⚠ Expected HOVERING_FOR_CORRECTION, got: {current_mode}")
    
    # Continue simulating pose during hover
    print(f"  Waiting for hover time ({SPRAYER_HOVER_TIME}s)...")
    for _ in range(int(SPRAYER_HOVER_TIME * 4) + 2):
        await simulate_pose(redis, target_wp["lat"], target_wp["lon"], SPRAYER_CRUISE_ALT)
        await asyncio.sleep(0.25)
    
    # ═══════════════════════════════════════════════════════════════════
    # STATE 3: DESCENDING_TO_SPRAY
    # ═══════════════════════════════════════════════════════════════════
    print("\n[State 3] DESCENDING_TO_SPRAY")
    
    if await wait_for_mode(redis, "DESCENDING_TO_SPRAY", timeout=5):
        print("  ✓ Entered DESCENDING_TO_SPRAY")
    else:
        current_mode = await get_sprayer_mode(redis)
        print(f"  ⚠ Expected DESCENDING_TO_SPRAY, got: {current_mode}")
    
    # Simulate descent
    print(f"  Simulating descent to {SPRAYER_SPRAY_ALT}m...")
    descent_steps = 5
    alt_step = (SPRAYER_CRUISE_ALT - SPRAYER_SPRAY_ALT) / descent_steps
    
    for i in range(descent_steps + 1):
        current_alt = SPRAYER_CRUISE_ALT - (i * alt_step)
        await simulate_pose(redis, target_wp["lat"], target_wp["lon"], current_alt)
        await asyncio.sleep(0.3)
        print(f"    Altitude: {current_alt:.1f}m")
    
    # ═══════════════════════════════════════════════════════════════════
    # STATE 4: SPRAYING
    # ═══════════════════════════════════════════════════════════════════
    print("\n[State 4] SPRAYING")
    
    if await wait_for_mode(redis, "SPRAYING", timeout=5):
        print("  ✓ Entered SPRAYING")
    else:
        current_mode = await get_sprayer_mode(redis)
        print(f"  ⚠ Expected SPRAYING, got: {current_mode}")
    
    # Simulate holding at spray altitude during spray
    print(f"  Spraying for {SPRAYER_SPRAY_DURATION}s...")
    for i in range(int(SPRAYER_SPRAY_DURATION * 4) + 2):
        await simulate_pose(redis, target_wp["lat"], target_wp["lon"], SPRAYER_SPRAY_ALT)
        await asyncio.sleep(0.25)
        if i % 4 == 0:
            print(f"    Spray time: {i/4:.1f}s")
    
    # ═══════════════════════════════════════════════════════════════════
    # STATE 5: ASCENDING_AFTER_SPRAY
    # ═══════════════════════════════════════════════════════════════════
    print("\n[State 5] ASCENDING_AFTER_SPRAY")
    
    if await wait_for_mode(redis, "ASCENDING_AFTER_SPRAY", timeout=5):
        print("  ✓ Entered ASCENDING_AFTER_SPRAY")
    else:
        current_mode = await get_sprayer_mode(redis)
        print(f"  ⚠ Expected ASCENDING_AFTER_SPRAY, got: {current_mode}")
    
    # Simulate ascent
    print(f"  Simulating ascent to {SPRAYER_CRUISE_ALT}m...")
    for i in range(descent_steps + 1):
        current_alt = SPRAYER_SPRAY_ALT + (i * alt_step)
        await simulate_pose(redis, target_wp["lat"], target_wp["lon"], current_alt)
        await asyncio.sleep(0.3)
        print(f"    Altitude: {current_alt:.1f}m")
    
    # ═══════════════════════════════════════════════════════════════════
    # STATE 6: IDLE (mission complete)
    # ═══════════════════════════════════════════════════════════════════
    print("\n[State 6] IDLE/HOVERING (crop complete)")
    
    await asyncio.sleep(2)
    current_mode = await get_sprayer_mode(redis)
    print(f"  Final mode: {current_mode}")
    
    # ═══════════════════════════════════════════════════════════════════
    # SUMMARY
    # ═══════════════════════════════════════════════════════════════════
    print("\n" + "=" * 70)
    print("STATE MACHINE TEST SUMMARY")
    print("=" * 70)
    
    crop_index = await redis.client.get("path_planner:current_crop_target_index")
    print(f"\n  Crops processed: {int(crop_index) + 1 if crop_index else 0}")
    print(f"  Final sprayer mode: {current_mode}")
    
    state_transitions = [
        "NAVIGATING_TO_WAYPOINT",
        "HOVERING_FOR_CORRECTION",
        "DESCENDING_TO_SPRAY",
        "SPRAYING",
        "ASCENDING_AFTER_SPRAY",
        "IDLE/HOVERING"
    ]
    print(f"\n  Expected state flow:")
    for i, state in enumerate(state_transitions):
        print(f"    {i+1}. {state}")
    
    print("\n" + "=" * 70)
    
    await redis.close()


if __name__ == "__main__":
    asyncio.run(main())
