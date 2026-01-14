import asyncio
import logging
import signal
import time
import math
import numpy as np
from common.redis_client import RedisClient
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from workers.lidar_processing.lidar_utils.conversion import gps_to_local, lidar_to_map

  
from workers.lidar_processing.lidar_utils.filtering import range_filter,std_filter,point_cloud_average,filter_dcscan
from workers.lidar_processing.lidar_utils.downsampling import angular_downsample



from matplotlib import pyplot as plt
from workers.lidar_processing.lidar_utils.occupancy_grid import map_to_grid,create_occupancy_grid,update_grid_from_scan,inflate_obstacles
from workers.SETTINGS import (
    WORKER_ID_LIDAR,
    LIDAR_GRID_SIZE,
    LIDAR_GRID_RESOLUTION,
    DIST_UNKNOWN_MM,
    DIST_MIN_MM,
    DIST_MAX_MM,
    PLOT_INTERVAL,
    INFLATION_RADIUS_CELLS,
    LAT0,
    LON0,
)

MAX_LIMIT=3

# def plot_scan(points):
#     plt.ion()
#     fig, ax = plt.subplots()    

#     xs, ys = [], []

#     for (x, y) in points:
#         xs.append(x)
#         ys.append(y)

#     ax.cla()  # 🔴 clear old scan

#     ax.scatter(xs, ys, s=5)
#     ax.scatter(0, 0, c='red', label="LiDAR")
#        # 🔒 LOCK AXIS LIMITS (this fixes size change)
#     ax.set_xlim(-MAX_LIMIT, MAX_LIMIT)
#     ax.set_ylim(-MAX_LIMIT, MAX_LIMIT)
#     ax.set_aspect('equal')
#     ax.grid()
#     ax.legend()

#     plt.draw()
#     plt.pause(0.001)  # allow UI refresh
#     plt.cla()


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

logger = logging.getLogger(__name__)

WORKER_ID = WORKER_ID_LIDAR

loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

shutdown_event = asyncio.Event()
redis = RedisClient(loop=loop, worker_id=WORKER_ID)

# Grid config
GRID_SIZE = LIDAR_GRID_SIZE
GRID_RESOLUTION = LIDAR_GRID_RESOLUTION

# Global state (used only as fallback - prefer Redis grid)
drone_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
time_boot_us = 0






# ====    plot the grid      ==== #
# Global plotting variables
# fig = None
# ax = None
# img = None
# robot_dot = None
# img_inflated = None
# robot_dot_inflated = None
# last_plot_time = 0



# def init_plots():
#     global fig, ax, img, robot_dot, img_inflated, robot_dot_inflated
#     if fig is not None:
#         return

#     plt.ion()
#     fig, ax = plt.subplots(1, 2, figsize=(8,8))
#     ax[0].set_xlim(0, occupancy_grid.shape[0])
#     ax[0].set_ylim(0, occupancy_grid.shape[1])
#     cmap = ListedColormap(["green", "red", "white"])

#     img = ax[0].imshow(occupancy_grid.T, cmap=cmap, origin='lower', interpolation='nearest')
#     gx,gy=map_to_grid(0,0)
#     robot_dot = ax[0].scatter(gx, gy,color='blue', marker='o', s=64)

#     img_inflated = ax[1].imshow(occupancy_grid.T, cmap=cmap, origin='lower', interpolation='nearest')
#     robot_dot_inflated = ax[1].scatter(gx, gy,color='blue', marker='o', s=64)
#     plt.show(block=False)



async def update_grid_async(x_d: float, y_d: float, yaw_deg: float,
                           distances_cm: list[int],
                           angle_offset_deg: float,
                           angle_inc_deg: float):
    """
    Update the occupancy grid with LiDAR scan data.
    Loads grid from Redis, updates it, and saves back.
    """
    # Load occupancy grid from Redis
    grid_raw = await redis.binary_client.get("occupancy_grid")
    
    if not grid_raw:
        logger.warning(f"[{WORKER_ID}] No occupancy grid in Redis, creating new one")
        occupancy_grid = create_occupancy_grid()
    else:
        occupancy_grid = np.frombuffer(grid_raw, dtype=np.uint8).reshape(
            (LIDAR_GRID_SIZE, LIDAR_GRID_SIZE)
        ).copy()  # Copy to make it writable

    distances_cm = np.array(distances_cm, dtype=np.float32)
    
    scan = [
        (d, angle_offset_deg + i * angle_inc_deg)
        for i, d in enumerate(distances_cm)
    ]
   
    point_locals = range_filter(scan, min_range=DIST_MIN_MM, max_range=DIST_MAX_MM)
    points_map = [lidar_to_map(x, y, x_d, y_d, yaw_deg) for (x, y) in point_locals]
    
    if len(points_map) > 0:
        update_grid_from_scan(points_map, occupancy_grid, (x_d, y_d, yaw_deg))
        
        # Save updated grid back to Redis
        await redis.binary_client.set("occupancy_grid", occupancy_grid.tobytes())
        
        logger.debug(f"[{WORKER_ID}] Grid updated with {len(points_map)} LiDAR points")





    



async def publish_grid():
    """Publish occupancy grid to Redis and emit update event"""
    await redis.binary_client.set("occupancy_grid", occupancy_grid.tobytes())
    await redis.publish("lidar_processing:occupancy_grid_updated", {"status": "updated", "timestamp": time.time()})
    logger.info(f"[{WORKER_ID}] Occupancy grid published")

@redis.listen("mission_manager:drone_pose_update")
async def handle_drone_pose_update(data):
    """Update drone pose from Mission Manager.
    
    Payload:
    {
        "drone_id": str,
        "lat": float,
        "lon": float,
        "alt_m": float,
        "timestamp": float
    }
    """
    global drone_pose
    
    # Only track scout drone for LiDAR mapping
    if data.get("drone_id") != "scout":
        return
    
    lat = data.get("lat")
    lon = data.get("lon")
    
    if lat is None or lon is None:
        return
    
    # Convert GPS to local ENU coordinates (meters)
    x, y = gps_to_local(lat, lon, LAT0, LON0)
    
    drone_pose["x"] = x
    drone_pose["y"] = y
    logger.debug(f"[{WORKER_ID}] Pose updated: x={x:.2f}m, y={y:.2f}m")

@redis.listen("mission_manager:drone_attitude_update")
async def handle_drone_yaw_update(data):
    """Update drone pose from Mission Manager"""
    global drone_pose
    drone_pose["yaw"] = data.get("yaw", 0.0)
    logger.debug(f"[{WORKER_ID}] Yaw updated: {drone_pose}")


@redis.listen("mission_manager:lidar_obstacle_distance")
async def handle_obstacle_distance(data):
    """Process OBSTACLE_DISTANCE message from drone"""
    global time_boot_us
    msg_time = data.get("time_usec", 0)
    if msg_time < time_boot_us:
        return

    distances = data.get("distances", [])
    angle_offset = data.get("angle_offset", 0)
    angle_inc = data.get("increment_f", 5)
    
    await update_grid_async(
        drone_pose["x"], drone_pose["y"], drone_pose["yaw"],
        distances, angle_offset, angle_inc
    )
    await publish_grid()


@redis.listen("mission_manager:system_time")
async def handle_system_time(data):
    """Update time sync"""
    global time_boot_us
    time_boot_us = data.get("time_boot_us", 0)

async def heartbeat_loop():
    while not shutdown_event.is_set():
        try:
            await redis.heartbeat()
        except Exception as e:
            logger.error(f"[{WORKER_ID}] Heartbeat error: {e}")
        await asyncio.sleep(1)


async def main(loop):
    await redis.connect()

    startup_mode = await redis.get_startup_mode()
    logger.info(f"[{WORKER_ID}] Starting in {startup_mode} mode")

    tasks = [
        loop.create_task(heartbeat_loop()),
    ]

    await shutdown_event.wait()

    logger.info(f"[{WORKER_ID}] Shutting down...")
    for task in tasks:
        task.cancel()

    await redis.close()


def handle_shutdown():
    logger.info(f"[{WORKER_ID}] Shutdown signal received")
    shutdown_event.set()


def runner():
    # Handle SIGINT / SIGTERM
    # for sig in (signal.SIGINT, signal.SIGTERM):
    #     loop.add_signal_handler(sig, handle_shutdown)


# for the plot
    # try:
    #     init_plots()
    # except Exception as e:
    #     logger.warning(f"Could not initialize plots: {e}")

    try:
        loop.run_until_complete(main(loop))
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"[{WORKER_ID}] Fatal error: {e}")
    finally:
        handle_shutdown()
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()


if __name__ == "__main__":
    runner()

    