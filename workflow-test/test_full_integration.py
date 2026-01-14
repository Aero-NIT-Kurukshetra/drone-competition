"""
Test 6: Full Integration Test
Runs both scout and sprayer workflows simultaneously.
"""
import asyncio
import json
import sys
import os
import time
import math
import numpy as np


from dotenv import load_dotenv

load_dotenv()
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.redis_client import RedisClient
from workers.SETTINGS import PATH_PLANNING_GRID_SIZE, GRID_FREE, GRID_UNEXPLORED

scout_loc = {
    "lat": None,
    "lon": None
}

# KML for scout
KML_XML = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
    <Placemark>
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


async def simulate_crop_detections(redis: RedisClient, stop_event: asyncio.Event):
    """
    Simulate crop detections when scout drone flies over crop locations.
    
    In real missions, the camera on the scout drone detects crops as it flies.
    This simulation monitors scout position and triggers detection events
    when the scout gets close to predefined "hidden" crop locations.
    """
    # Hidden crop locations that scout will "discover" when flying over
    hidden_crops = [
        {"lat": 29.9496992, "lon": 76.8163962, "detected": False},
        {"lat": 29.94980, "lon": 76.81620, "detected": False},
        {"lat": 29.95010, "lon": 76.81640, "detected": False},
        {"lat": 29.95040, "lon": 76.81660, "detected": False},
    ]
    
    DETECTION_RADIUS_M = 10.0  # Scout must be within 10m to detect crop
    EARTH_RADIUS = 6378137.0
    
    def haversine_distance(lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS points in meters."""
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        return EARTH_RADIUS * c
    
    detected_count = 0
    
    while not stop_event.is_set():
        # Get current scout position from mission state
        state_raw = await redis.client.get("mission:state")
        if state_raw:
            state = json.loads(state_raw)
            scout = state["drones"]["scout"]
            
            # Only check when scout is actively flying
            if scout["mode"] in ["NAVIGATING_TO_WAYPOINT", "REACHED_WAYPOINT"]:
                scout_lat = scout_loc['lat']
                scout_lon = scout_loc['lon']

                if scout_lat and scout_lon:
                    # Check proximity to each hidden crop
                    for i, crop in enumerate(hidden_crops):
                        if crop["detected"]:
                            continue
                            
                        distance = haversine_distance(
                            scout_lat, scout_lon,
                            crop["lat"], crop["lon"]
                        )
                        
                        if distance <= DETECTION_RADIUS_M:
                            crop["detected"] = True
                            detected_count += 1
                            
                            print(f"\n  🌱 [Scout Camera] Crop detected at lat={crop['lat']}, lon={crop['lon']} (distance: {distance:.1f}m)")
                            
                            await redis.publish("event:crop_detected", {
                                "lat": crop["lat"],
                                "lon": crop["lon"],
                                "confidence": 0.85 + (detected_count * 0.05)
                            })
        
        # Check if all crops detected
        if all(c["detected"] for c in hidden_crops):
            print(f"\n  ✓ All {len(hidden_crops)} crops detected by scout camera")
            break
            
        await asyncio.sleep(0.5)


async def monitor_state(redis: RedisClient, stop_event: asyncio.Event):
    """Monitor and display drone states."""
    last_scout_mode = None
    last_sprayer_mode = None
    
    while not stop_event.is_set():
        state_raw = await redis.client.get("mission:state")
        if state_raw:
            state = json.loads(state_raw)
            scout = state["drones"]["scout"]
            sprayer = state["drones"]["sprayer"]
            
            # Detect mode changes
            if scout["mode"] != last_scout_mode:
                print(f"\n  🚁 Scout: {scout['mode']}")
                last_scout_mode = scout["mode"]
                
            if sprayer["mode"] != last_sprayer_mode:
                print(f"\n  🚜 Sprayer: {sprayer['mode']}")
                last_sprayer_mode = sprayer["mode"]
            
            # Check for mission completion
            if scout["mode"] == "LANDING" and sprayer["mode"] == "IDLE":
                crop_locations = await redis.client.get("crop_locations")
                crop_index = await redis.client.get("path_planner:current_crop_target_index")
                
                if crop_locations and crop_index:
                    crops = json.loads(crop_locations)
                    idx = int(crop_index)
                    
                    if idx >= len(crops) - 1:
                        print("\n\n✓ MISSION COMPLETE - All waypoints and crops processed!")
                        stop_event.set()
        
        await asyncio.sleep(0.5)


async def main():
    loop = asyncio.get_event_loop()
    redis = RedisClient(loop=loop, worker_id="test_full_integration")

    @redis.listen("mission_manager:drone_pose_update")
    async def handle_scout_pose_update(data):
        """Capture scout drone pose updates."""
        if data.get("drone_id") == "scout":
            scout_loc["lat"] = data.get("lat")
            scout_loc["lon"] = data.get("lon")
    
    await redis.connect()
    print("✓ Connected to Redis")
    
    # Clear all previous data
    keys_to_clear = [
        "crop_locations",
        "path_planner:current_crop_target_index",
        "path_planner:scout_waypoints",
        "path_planner:current_scout_waypoint_index",
        "path_planner:sprayer_waypoints",
        "path_planner:current_sprayer_waypoint_index",
        "mission:state"
    ]
    
    for key in keys_to_clear:
        await redis.client.delete(key)
    
    # Initialize crop index to -1 (no crops processed yet)
    await redis.client.set("path_planner:current_crop_target_index", "-1")
    print("✓ Cleared previous mission data")
    
    # Create fully explored (free) occupancy grid for integration test
    # This allows sprayer to plan paths without waiting for exploration
    # (Fog-of-war mechanics are tested separately in test_exploration_wait.py)
    grid = np.full((PATH_PLANNING_GRID_SIZE, PATH_PLANNING_GRID_SIZE), GRID_FREE, dtype=np.uint8)
    await redis.binary_client.set("occupancy_grid", grid.tobytes())
    print("✓ Created fully explored occupancy grid (no obstacles)")
    
    print("\n" + "=" * 70)
    print("FULL INTEGRATION TEST")
    print("=" * 70)
    print("""
This test simulates a complete mission:

  1. Scout generates lawnmower waypoints from KML
  2. Scout arms, takes off, and navigates the pattern
  3. As scout flies over crop locations, camera detects them
  4. Sprayer arms, takes off, and navigates to crops
  5. Both drones operate concurrently

Note: 
  - Grid is fully explored (no fog-of-war) for this integration test
  - Crops are detected when scout is within 10m of crop location
  - For fog-of-war testing, see test_exploration_wait.py

Prerequisites:
  - Redis running
  - Scout SITL on port 13550
  - Sprayer SITL on port 13560
  - Mission Manager running
  - Path Planner running
""")
    
    input("Press ENTER to start full integration test...")
    
    # Step 1: Generate scout waypoints
    print("\n[1] Generating scout waypoints...")
    await redis.publish("mission_manager:scout_planning_request", {
        "kml_xml": KML_XML,
        "spacing": 15,  # Larger spacing for faster test
        "angle": 45
    })
    await asyncio.sleep(2)
    
    waypoints_raw = await redis.client.get("path_planner:scout_waypoints")
    if not waypoints_raw:
        print("✗ Failed to generate waypoints!")
        await redis.close()
        return
    
    data = json.loads(waypoints_raw)
    print(f"✓ Generated {len(data['waypoints'])} scout waypoints")
    
    # Step 2: Start mission
    print("\n[2] Starting mission...")
    await redis.publish("start_mission", {})
    
    # Step 3: Run concurrent tasks
    print("\n[3] Running integration test...")
    print("-" * 70)
    
    stop_event = asyncio.Event()
    
    try:
        await asyncio.gather(
            monitor_state(redis, stop_event),
            simulate_crop_detections(redis, stop_event),
        )
    except KeyboardInterrupt:
        print("\n\nTest interrupted.")
        stop_event.set()
    
    # Final report
    print("\n" + "=" * 70)
    print("INTEGRATION TEST SUMMARY")
    print("=" * 70)
    
    state_raw = await redis.client.get("mission:state")
    if state_raw:
        state = json.loads(state_raw)
        print(f"\n  Scout final mode: {state['drones']['scout']['mode']}")
        print(f"  Sprayer final mode: {state['drones']['sprayer']['mode']}")
    
    scout_wp = await redis.client.get("path_planner:current_scout_waypoint_index")
    scout_total_raw = await redis.client.get("path_planner:scout_waypoints")
    if scout_total_raw:
        scout_total = len(json.loads(scout_total_raw)["waypoints"])
        print(f"\n  Scout waypoints: {scout_wp}/{scout_total}")
    
    crops_raw = await redis.client.get("crop_locations")
    crop_index = await redis.client.get("path_planner:current_crop_target_index")
    if crops_raw:
        crops = json.loads(crops_raw)
        processed = int(crop_index) + 1 if crop_index and int(crop_index) >= 0 else 0
        print(f"  Crops detected: {len(crops)}")
        print(f"  Crops sprayed: {processed}")
    
    print("\n" + "=" * 70)
    await redis.close()


if __name__ == "__main__":
    asyncio.run(main())
