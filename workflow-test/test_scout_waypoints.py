"""
Test 1: Scout Lawnmower Pattern Generation
Generates waypoints from KML without flying.
"""
import asyncio
import json
import sys
import os

from dotenv import load_dotenv

load_dotenv()

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.redis_client import RedisClient

# KML content from NIDAR.kml
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
    redis = RedisClient(loop=loop, worker_id="test_scout_waypoints")
    
    await redis.connect()
    print("✓ Connected to Redis")
    
    # Publish scout planning request
    payload = {
        "kml_xml": KML_XML,
        "spacing": 5,  # meters between lines
        "angle": 60    # sweep angle in degrees
    }
    
    print("\n📤 Publishing scout planning request...")
    await redis.publish("mission_manager:scout_planning_request", payload)
    
    # Wait for path planner to process
    await asyncio.sleep(2)
    
    # Check if waypoints were generated
    waypoints_raw = await redis.client.get("path_planner:scout_waypoints")
    
    if waypoints_raw:
        data = json.loads(waypoints_raw)
        waypoints = data["waypoints"]
        
        print(f"\n✓ Generated {len(waypoints)} waypoints:\n")
        print("-" * 60)
        for i, wp in enumerate(waypoints, 1):
            print(f"  {i:03d}: lat={wp['lat']:.7f}, lon={wp['lon']:.7f}, alt={wp['alt_m']}m")
        print("-" * 60)
        
        # Check index
        index = await redis.client.get("path_planner:current_scout_waypoint_index")
        print(f"\n✓ Current waypoint index: {index}")
    else:
        print("\n✗ No waypoints generated!")
        print("  Make sure path_planning worker is running.")
    
    await redis.close()


if __name__ == "__main__":
    asyncio.run(main())
