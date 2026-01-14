"""
Test 5: Full Sprayer Workflow
Tests complete sprayer workflow including arm, takeoff, navigate, and spray.

Uses coordinates inside the farm polygon from NIDAR.kml.
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
    PATH_PLANNING_MAP_SIZE, 
    PATH_PLANNING_RESOLUTION
)

# ============================================
# FARM POLYGON (from NIDAR.kml) for reference
# ============================================
# Bounds: lat 29.9494-29.9508, lon 76.8159-76.8167

# Sprayer home position (southwest area of polygon)
SPRAYER_HOME = {
    "lat": 29.94965,
    "lon": 76.81595,
    "alt": 5.0
}

# Multiple crop locations distributed inside the polygon
MOCK_CROPS = [
    {"lat": 29.94980, "lon": 76.81620},  # South-center
    {"lat": 29.95010, "lon": 76.81640},  # Center
    {"lat": 29.95040, "lon": 76.81655},  # North-center
    {"lat": 29.95055, "lon": 76.81625},  # North-west
    {"lat": 29.94995, "lon": 76.81605},  # West-center
]


def gps_to_grid(lat, lon):
    """Convert GPS to grid coordinates."""
    import math
    EARTH_RADIUS = 6378137.0
    
    dlat = math.radians(lat - LAT0)
    dlon = math.radians(lon - LON0)
    
    x = dlon * math.cos(math.radians(LAT0)) * EARTH_RADIUS
    y = dlat * EARTH_RADIUS
    
    gx = int((x + PATH_PLANNING_MAP_SIZE / 2) / PATH_PLANNING_RESOLUTION)
    gy = int((y + PATH_PLANNING_MAP_SIZE / 2) / PATH_PLANNING_RESOLUTION)
    
    return gx, gy


def create_explored_grid(grid_size: int) -> np.ndarray:
    """Create a fully explored occupancy grid with some obstacles."""
    grid = np.zeros((grid_size, grid_size), dtype=np.uint8)
    
    # Add obstacle outside the main flight area (east boundary)
    gx1, gy1 = gps_to_grid(29.95020, 76.81685)
    for dx in range(-4, 5):
        for dy in range(-15, 16):
            gx, gy = gx1 + dx, gy1 + dy
            if 0 <= gx < grid_size and 0 <= gy < grid_size:
                grid[gy, gx] = 1
    
    # Small obstacle that A* should navigate around
    gx2, gy2 = gps_to_grid(29.95015, 76.81632)
    for dx in range(-2, 3):
        for dy in range(-2, 3):
            gx, gy = gx2 + dx, gy2 + dy
            if 0 <= gx < grid_size and 0 <= gy < grid_size:
                grid[gy, gx] = 1
    
    return grid


async def main():
    loop = asyncio.get_event_loop()
    redis = RedisClient(loop=loop, worker_id="test_sprayer_mission")
    
    await redis.connect()
    print("✓ Connected to Redis")
    
    # Clear previous data
    await redis.client.delete("crop_locations")
    await redis.client.delete("path_planner:current_crop_target_index")
    await redis.client.delete("path_planner:sprayer_waypoints")
    await redis.client.delete("path_planner:current_sprayer_waypoint_index")
    
    # Store explored occupancy grid with obstacles
    print("\n[Setup] Creating occupancy grid...")
    grid = create_explored_grid(PATH_PLANNING_GRID_SIZE)
    obstacle_count = np.sum(grid == 1)
    await redis.binary_client.set("occupancy_grid", grid.tobytes())
    print(f"✓ Created occupancy grid ({PATH_PLANNING_GRID_SIZE}x{PATH_PLANNING_GRID_SIZE}, {obstacle_count} obstacle cells)")
    
    # Store crop locations
    await redis.client.set("crop_locations", json.dumps(MOCK_CROPS))
    await redis.client.set("path_planner:current_crop_target_index", "-1")
    print(f"✓ Stored {len(MOCK_CROPS)} crop locations")
    
    print("\n" + "=" * 70)
    print("SPRAYER WORKFLOW TEST")
    print("=" * 70)
    
    print(f"\nConfiguration:")
    print(f"  Origin (LAT0, LON0): ({LAT0}, {LON0})")
    print(f"  Map size: {PATH_PLANNING_MAP_SIZE}m x {PATH_PLANNING_MAP_SIZE}m")
    print(f"  Sprayer home: lat={SPRAYER_HOME['lat']}, lon={SPRAYER_HOME['lon']}")
    
    print(f"\nCrop locations ({len(MOCK_CROPS)} total):")
    for i, crop in enumerate(MOCK_CROPS, 1):
        grid_pos = gps_to_grid(crop["lat"], crop["lon"])
        print(f"  {i}. lat={crop['lat']:.5f}, lon={crop['lon']:.5f} → grid {grid_pos}")
    
    print("\nWorkflow steps:")
    print("  1. Start mission → Initialize")
    print("  2. Set sprayer location → Enable dispatch")
    print("  3. Detect crops → Trigger planning")
    print("  4. Navigate → Spray → Next crop → Repeat")
    
    input("\nPress ENTER to start (ensure Mission Manager + Path Planner are running)...")
    
    # Step 1: Start mission
    print("\n[1] Starting mission...")
    await redis.publish("start_mission", {})
    await asyncio.sleep(3)
    print("  ✓ Mission started")
    
    # Step 2: Set sprayer initial location
    print("\n[2] Setting sprayer initial location...")
    await redis.publish("mission_manager:drone_pose_update", {
        "drone_id": "sprayer",
        "lat": SPRAYER_HOME["lat"],
        "lon": SPRAYER_HOME["lon"],
        "alt_m": SPRAYER_HOME["alt"],
        "timestamp": time.time()
    })
    await asyncio.sleep(1)
    print(f"  ✓ Sprayer at: lat={SPRAYER_HOME['lat']}, lon={SPRAYER_HOME['lon']}")
    
    # Step 3: Detect crops (this triggers sprayer planning)
    print(f"\n[3] Simulating crop detection ({len(MOCK_CROPS)} crops)...")
    for i, crop in enumerate(MOCK_CROPS):
        await redis.publish("event:crop_detected", {
            "lat": crop["lat"],
            "lon": crop["lon"],
            "confidence": 0.9
        })
        print(f"  ✓ Crop {i+1} detected at ({crop['lat']:.5f}, {crop['lon']:.5f})")
        await asyncio.sleep(0.3)
    
    # Wait for planning to complete
    await asyncio.sleep(2)
    
    # Check if waypoints were generated
    waypoints_raw = await redis.client.get("path_planner:sprayer_waypoints")
    if waypoints_raw:
        data = json.loads(waypoints_raw)
        print(f"\n  ✓ Path planning generated {len(data['waypoints'])} waypoints to first crop")
    else:
        print("\n  ⚠ No waypoints generated - check path_planning worker logs")
    
    # Step 4: Monitor sprayer state
    print("\n[4] Monitoring sprayer workflow (Ctrl+C to stop)...")
    print("-" * 70)
    
    last_mode = None
    last_crop_index = -1
    
    try:
        while True:
            state_raw = await redis.client.get("mission:state")
            crop_index_raw = await redis.client.get("path_planner:current_crop_target_index")
            current_crop_index = int(crop_index_raw) if crop_index_raw else -1
            
            if state_raw:
                state = json.loads(state_raw)
                sprayer = state["drones"]["sprayer"]
                current_mode = sprayer["mode"]
                
                # Print state changes
                if current_mode != last_mode:
                    print(f"\n  [{time.strftime('%H:%M:%S')}] Sprayer Mode: {current_mode}")
                    last_mode = current_mode
                
                # Print crop progress
                if current_crop_index != last_crop_index and current_crop_index >= 0:
                    print(f"  [{time.strftime('%H:%M:%S')}] Processing crop {current_crop_index + 1}/{len(MOCK_CROPS)}")
                    last_crop_index = current_crop_index
                
                # Show position updates
                loc = sprayer.get("current_loc", {})
                if loc.get("lat"):
                    sys.stdout.write(
                        f"\r  Position: lat={loc['lat']:.6f}, lon={loc['lon']:.6f}, "
                        f"alt={loc.get('alt', 0):.1f}m    "
                    )
                    sys.stdout.flush()
                
                # Check if all crops processed
                if current_mode == "IDLE" and current_crop_index >= len(MOCK_CROPS) - 1:
                    print(f"\n\n  ✓ All {len(MOCK_CROPS)} crops processed!")
                    break
            
            await asyncio.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n\nMonitoring stopped by user.")
    
    # Final state summary
    print("\n" + "=" * 70)
    print("FINAL STATE")
    print("=" * 70)
    
    crop_index = await redis.client.get("path_planner:current_crop_target_index")
    crops_done = int(crop_index) + 1 if crop_index else 0
    print(f"  Crops processed: {crops_done}/{len(MOCK_CROPS)}")
    
    state_raw = await redis.client.get("mission:state")
    if state_raw:
        state = json.loads(state_raw)
        sprayer = state["drones"]["sprayer"]
        print(f"  Sprayer mode: {sprayer['mode']}")
        loc = sprayer.get("current_loc", {})
        if loc.get("lat"):
            print(f"  Final position: lat={loc['lat']:.6f}, lon={loc['lon']:.6f}")
    
    await redis.close()


if __name__ == "__main__":
    asyncio.run(main())
