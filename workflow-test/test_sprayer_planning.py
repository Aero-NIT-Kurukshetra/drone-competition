"""
Test 4: Sprayer Path Planning (A*)
Tests A* path planning from sprayer to crop location.

Uses coordinates inside the farm polygon from NIDAR.kml:
Polygon vertices:
  76.81589155096442,29.94965311567124
  76.8162492257966,29.94939808518254
  76.81664355584154,29.94963310749574
  76.81673785885653,29.95054251583795
  76.81637533963627,29.95078200571026
  76.81601151843628,29.95060813092637
"""
import asyncio
import json
import sys
import os
import numpy as np

from dotenv import load_dotenv

load_dotenv()

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.redis_client import RedisClient
from workers.SETTINGS import PATH_PLANNING_GRID_SIZE, LAT0, LON0, PATH_PLANNING_MAP_SIZE, PATH_PLANNING_RESOLUTION

# ============================================
# FARM POLYGON COORDINATES (from NIDAR.kml)
# ============================================
FARM_POLYGON = [
    {"lon": 76.81589155096442, "lat": 29.94965311567124},
    {"lon": 76.8162492257966,  "lat": 29.94939808518254},
    {"lon": 76.81664355584154, "lat": 29.94963310749574},
    {"lon": 76.81673785885653, "lat": 29.95054251583795},
    {"lon": 76.81637533963627, "lat": 29.95078200571026},
    {"lon": 76.81601151843628, "lat": 29.95060813092637},
]

# Sprayer starting position (southwest corner of polygon)
MOCK_SPRAYER_POSE = {
    "lat": 29.94965,      # Southwest corner
    "lon": 76.81595,
    "alt": 5.0
}

# Multiple crop locations distributed across the polygon
MOCK_CROP_LOCATIONS = [
    {"lat": 29.94980, "lon": 76.81620},  # Center-south
    {"lat": 29.95010, "lon": 76.81640},  # Center
    {"lat": 29.95040, "lon": 76.81660},  # Center-north
    {"lat": 29.95060, "lon": 76.81630},  # North
    {"lat": 29.95000, "lon": 76.81610},  # Center-west
]


def gps_to_grid(lat, lon):
    """Convert GPS to grid coordinates (same logic as path_planning)."""
    import math
    EARTH_RADIUS = 6378137.0
    
    dlat = math.radians(lat - LAT0)
    dlon = math.radians(lon - LON0)
    
    x = dlon * math.cos(math.radians(LAT0)) * EARTH_RADIUS
    y = dlat * EARTH_RADIUS
    
    gx = int((x + PATH_PLANNING_MAP_SIZE / 2) / PATH_PLANNING_RESOLUTION)
    gy = int((y + PATH_PLANNING_MAP_SIZE / 2) / PATH_PLANNING_RESOLUTION)
    
    return gx, gy


# Grid cell values
GRID_FREE = 0
GRID_OBSTACLE = 1
GRID_UNEXPLORED = 2


def create_explored_occupancy_grid(grid_size: int) -> np.ndarray:
    """
    Create a fully explored occupancy grid with realistic obstacles.
    Grid is centered at LAT0, LON0 (center of farm polygon).
    
    Cell values:
        0 = explored, free
        1 = explored, obstacle
        2 = unexplored
    """
    grid = np.zeros((grid_size, grid_size), dtype=np.uint8)
    
    # Add some realistic obstacles (value = 1)
    # These represent trees, structures, etc. outside the main path
    
    # Obstacle cluster 1: Tree line on east side
    gx1, gy1 = gps_to_grid(29.95030, 76.81680)  # East of polygon
    for dx in range(-3, 4):
        for dy in range(-10, 11):
            gx, gy = gx1 + dx, gy1 + dy
            if 0 <= gx < grid_size and 0 <= gy < grid_size:
                grid[gy, gx] = GRID_OBSTACLE
    
    # Obstacle cluster 2: Structure in southwest
    gx2, gy2 = gps_to_grid(29.94955, 76.81570)  # Southwest outside polygon
    for dx in range(-5, 6):
        for dy in range(-5, 6):
            gx, gy = gx2 + dx, gy2 + dy
            if 0 <= gx < grid_size and 0 <= gy < grid_size:
                grid[gy, gx] = GRID_OBSTACLE
    
    # Obstacle cluster 3: Small obstacle in center (drone should navigate around)
    gx3, gy3 = gps_to_grid(29.95000, 76.81630)
    for dx in range(-2, 3):
        for dy in range(-2, 3):
            gx, gy = gx3 + dx, gy3 + dy
            if 0 <= gx < grid_size and 0 <= gy < grid_size:
                grid[gy, gx] = GRID_OBSTACLE
    
    return grid


def create_partially_explored_grid(grid_size: int, explored_radius_cells: int = 40) -> np.ndarray:
    """
    Create a grid that starts mostly unexplored, with only the area around 
    the sprayer's starting position explored (simulating scout already flew there).
    """
    # Start with all unexplored
    grid = np.full((grid_size, grid_size), GRID_UNEXPLORED, dtype=np.uint8)
    
    # Mark area around sprayer start as explored
    sprayer_gx, sprayer_gy = gps_to_grid(MOCK_SPRAYER_POSE["lat"], MOCK_SPRAYER_POSE["lon"])
    
    for dx in range(-explored_radius_cells, explored_radius_cells + 1):
        for dy in range(-explored_radius_cells, explored_radius_cells + 1):
            if dx*dx + dy*dy <= explored_radius_cells*explored_radius_cells:
                gx, gy = sprayer_gx + dx, sprayer_gy + dy
                if 0 <= gx < grid_size and 0 <= gy < grid_size:
                    grid[gy, gx] = GRID_FREE
    
    return grid


async def main():
    loop = asyncio.get_event_loop()
    redis = RedisClient(loop=loop, worker_id="test_sprayer_planning")
    
    await redis.connect()
    print("✓ Connected to Redis")
    
    print("\n" + "=" * 70)
    print("SPRAYER PATH PLANNING TEST (A*)")
    print("=" * 70)
    
    print(f"\nGrid Configuration:")
    print(f"  Origin (LAT0, LON0): ({LAT0}, {LON0})")
    print(f"  Map size: {PATH_PLANNING_MAP_SIZE}m x {PATH_PLANNING_MAP_SIZE}m")
    print(f"  Resolution: {PATH_PLANNING_RESOLUTION}m/cell")
    print(f"  Grid size: {PATH_PLANNING_GRID_SIZE}x{PATH_PLANNING_GRID_SIZE}")
    
    # Step 1: Create and store mock occupancy grid
    print("\n[Step 1] Creating explored occupancy grid with obstacles...")
    grid = create_explored_occupancy_grid(PATH_PLANNING_GRID_SIZE)
    obstacle_count = np.sum(grid == 1)
    print(f"  ✓ Grid created: {PATH_PLANNING_GRID_SIZE}x{PATH_PLANNING_GRID_SIZE}")
    print(f"  ✓ Obstacle cells: {obstacle_count}")
    
    # Show sprayer and crop grid positions
    sprayer_grid = gps_to_grid(MOCK_SPRAYER_POSE["lat"], MOCK_SPRAYER_POSE["lon"])
    print(f"  ✓ Sprayer grid position: {sprayer_grid}")
    
    await redis.binary_client.set("occupancy_grid", grid.tobytes())
    print("  ✓ Stored occupancy grid in Redis")
    
    # Step 2: Store crop locations
    print(f"\n[Step 2] Setting up {len(MOCK_CROP_LOCATIONS)} crop locations...")
    await redis.client.set("crop_locations", json.dumps(MOCK_CROP_LOCATIONS))
    await redis.client.set("path_planner:current_crop_target_index", "-1")
    
    for i, crop in enumerate(MOCK_CROP_LOCATIONS):
        crop_grid = gps_to_grid(crop["lat"], crop["lon"])
        print(f"  Crop {i+1}: lat={crop['lat']:.5f}, lon={crop['lon']:.5f} → grid {crop_grid}")
    
    # Step 3: Request sprayer planning
    print("\n[Step 3] Requesting sprayer path planning...")
    print(f"  Sprayer at: lat={MOCK_SPRAYER_POSE['lat']}, lon={MOCK_SPRAYER_POSE['lon']}")
    
    payload = {
        "drone_id": "sprayer",
        "drone_pose": MOCK_SPRAYER_POSE
    }
    
    await redis.publish("mission_manager:sprayer_plan_request", payload)
    print("  ✓ Published sprayer_plan_request")
    
    # Wait for planning
    print("  Waiting for path planning...")
    await asyncio.sleep(3)
    
    # Step 4: Check results
    print("\n[Step 4] Checking results...")
    
    sprayer_waypoints_raw = await redis.client.get("path_planner:sprayer_waypoints")
    
    if sprayer_waypoints_raw:
        data = json.loads(sprayer_waypoints_raw)
        waypoints = data["waypoints"]
        
        print(f"\n✓ SUCCESS: Generated {len(waypoints)} sprayer waypoints\n")
        print("-" * 70)
        print(f"{'#':>4}  {'Latitude':>12}  {'Longitude':>12}  {'Alt':>6}  {'Grid Pos':>12}")
        print("-" * 70)
        for i, wp in enumerate(waypoints, 1):
            wp_grid = gps_to_grid(wp["lat"], wp["lon"])
            print(f"{i:4d}  {wp['lat']:12.7f}  {wp['lon']:12.7f}  {wp['alt_m']:5.1f}m  {str(wp_grid):>12}")
        print("-" * 70)
        
        # Check indices
        wp_index = await redis.client.get("path_planner:current_sprayer_waypoint_index")
        crop_index = await redis.client.get("path_planner:current_crop_target_index")
        print(f"\n✓ Current sprayer waypoint index: {wp_index}")
        print(f"✓ Current crop target index: {crop_index}")
        
    else:
        print("\n✗ FAILED: No sprayer waypoints generated!")
        print("  Possible causes:")
        print("  - Path planning worker not running")
        print("  - Start/goal position out of grid bounds")
        print("  - No valid path found (blocked by obstacles)")
        
        # Debug info
        crop_index = await redis.client.get("path_planner:current_crop_target_index")
        print(f"\n  Debug: current_crop_target_index = {crop_index}")
    
    print("\n" + "=" * 70)
    await redis.close()


if __name__ == "__main__":
    asyncio.run(main())
