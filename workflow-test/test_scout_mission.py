"""
Test 2: Full Scout Mission
Starts complete scout mission with SITL.
"""
import asyncio
import json
import sys
import os

from dotenv import load_dotenv

load_dotenv()

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.redis_client import RedisClient

# KML content
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


async def main():
    loop = asyncio.get_event_loop()
    redis = RedisClient(loop=loop, worker_id="test_scout_mission")
    
    await redis.connect()
    print("✓ Connected to Redis")
    
    # Step 1: Generate waypoints first
    print("\n[Step 1] Generating lawnmower waypoints...")
    
    payload = {
        "kml_xml": KML_XML,
        "spacing": 10,  # larger spacing for faster test
        "angle": 45
    }
    
    await redis.publish("mission_manager:scout_planning_request", payload)
    await asyncio.sleep(2)
    
    waypoints_raw = await redis.client.get("path_planner:scout_waypoints")
    if not waypoints_raw:
        print("✗ Failed to generate waypoints. Is path_planning worker running?")
        await redis.close()
        return
    
    data = json.loads(waypoints_raw)
    print(f"✓ Generated {len(data['waypoints'])} waypoints")
    
    # Step 2: Start mission
    print("\n[Step 2] Starting mission...")
    print("  This will:")
    print("  - Set scout to GUIDED mode")
    print("  - Arm and takeoff to 5m")
    print("  - Navigate through waypoints")
    print("  - Land when complete")
    
    input("\nPress ENTER to start mission (Ctrl+C to cancel)...")
    
    await redis.publish("start_mission", {})
    
    print("\n✓ Mission started!")
    print("\nMonitoring mission state (Ctrl+C to stop)...")
    print("-" * 60)
    
    # Monitor mission progress
    try:
        while True:
            state_raw = await redis.client.get("mission:state")
            if state_raw:
                state = json.loads(state_raw)
                scout = state["drones"]["scout"]
                
                print(f"\r  Scout Mode: {scout['mode']:25} | "
                      f"WP: {scout.get('active_waypoint', 'None')}", end="")
            
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("\n\nMonitoring stopped.")
    
    await redis.close()


if __name__ == "__main__":
    asyncio.run(main())
