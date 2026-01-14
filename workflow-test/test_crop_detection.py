"""
Test 3: Crop Detection Simulation
Simulates crop detection events to test sprayer trigger.
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

# Mock crop locations (within the farm polygon)
MOCK_CROPS = [
    {"lat": 29.94980, "lon": 76.81620, "confidence": 0.95},
    {"lat": 29.95010, "lon": 76.81640, "confidence": 0.87},
    {"lat": 29.95040, "lon": 76.81650, "confidence": 0.92},
]

# Mock sprayer initial location
MOCK_SPRAYER_LOCATION = {
    "lat": 29.94965,
    "lon": 76.81590,
    "alt": 5.0
}


async def main():
    loop = asyncio.get_event_loop()
    redis = RedisClient(loop=loop, worker_id="test_crop_detection")
    
    await redis.connect()
    print("✓ Connected to Redis")
    
    # Clear existing crop locations
    await redis.client.delete("crop_locations")
    await redis.client.delete("path_planner:current_crop_target_index")
    print("✓ Cleared existing crop data")
    
    # Set up sprayer's initial location (simulate pose update)
    print("\n[Setup] Setting sprayer initial location...")
    await redis.publish("mission_manager:drone_pose_update", {
        "drone_id": "sprayer",
        "lat": MOCK_SPRAYER_LOCATION["lat"],
        "lon": MOCK_SPRAYER_LOCATION["lon"],
        "alt_m": MOCK_SPRAYER_LOCATION["alt"],
        "timestamp": time.time()
    })
    print(f"  Sprayer at: lat={MOCK_SPRAYER_LOCATION['lat']}, lon={MOCK_SPRAYER_LOCATION['lon']}")
    await asyncio.sleep(0.5)  # Give time for mission manager to process
    
    print("\n" + "=" * 60)
    print("CROP DETECTION SIMULATION")
    print("=" * 60)
    
    for i, crop in enumerate(MOCK_CROPS, 1):
        print(f"\n[Crop {i}/{len(MOCK_CROPS)}]")
        print(f"  Location: lat={crop['lat']}, lon={crop['lon']}")
        print(f"  Confidence: {crop['confidence']:.0%}")
        
        # Simulate crop detection event
        event = {
            "lat": crop["lat"],
            "lon": crop["lon"],
            "confidence": crop["confidence"],
            "timestamp": time.time()
        }
        
        await redis.publish("event:crop_detected", event)
        print("  ✓ Published crop_detected event")
        
        await asyncio.sleep(0.5)
    
    # Verify crops stored
    await asyncio.sleep(1)
    
    crop_locations_raw = await redis.client.get("crop_locations")
    if crop_locations_raw:
        crops = json.loads(crop_locations_raw)
        print(f"\n✓ Stored {len(crops)} crop locations in Redis")
        
        for i, c in enumerate(crops, 1):
            print(f"  {i}: lat={c['lat']}, lon={c['lon']}")
    else:
        print("\n⚠ No crops stored - Mission Manager may not be running")
    
    # Check current target index
    index = await redis.client.get("path_planner:current_crop_target_index")
    print(f"\n  Current crop target index: {index}")
    
    # Check sprayer state
    state_raw = await redis.client.get("mission:state")
    if state_raw:
        state = json.loads(state_raw)
        sprayer = state["drones"]["sprayer"]
        print(f"\n  Sprayer mode: {sprayer['mode']}")
    
    print("\n" + "=" * 60)
    await redis.close()


if __name__ == "__main__":
    asyncio.run(main())
