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

WORKER_ID = "lidar_worker"

loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

shutdown_event = asyncio.Event()
redis = RedisClient(loop=loop, worker_id=WORKER_ID)

# Grid config
GRID_SIZE = 65
GRID_RESOLUTION = 0.1  # meters per cell
DIST_UNKNOWN_MM = 65535
DIST_MIN_MM = 20
DIST_MAX_MM = 1400

# Global state
occupancy_grid = create_occupancy_grid()
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


PLOT_INTERVAL = 0.2  # Update plot max 5 times per second



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



def update_grid(x_d: float, y_d: float, yaw_deg: float,
                distances_cm: list[int],
                angle_offset_deg: float,
                angle_inc_deg: float):
    global occupancy_grid, last_plot_time

    # Initialize here if running standalone and not via runner (though runner is preferred)
    # or just return if not initialized

    distances_cm = np.array(distances_cm, dtype=np.float32)
    
    
    scan=[
        (d, angle_offset_deg + i * angle_inc_deg)
        for i, d in enumerate(distances_cm)
    ]
    # print(scan)
    # print("angle increment:",angle_inc_deg," angle offset:",angle_offset_deg)
   
    point_locals=range_filter(scan,min_range=DIST_MIN_MM,max_range=DIST_MAX_MM)

    # for point in point_locals:
    #     print(f"Point before filtering: {point}")
    # print(f"LiDAR points after range filter: {len(point_locals)}")

    points_map=[lidar_to_map(x,y,x_d,y_d,yaw_deg) for (x,y) in point_locals]


    # plot_scan(points_map)
    # print(f"Updating occupancy grid with {len(points_map)} points")
    
    update_grid_from_scan(points_map, occupancy_grid, (x_d, y_d, yaw_deg))
    
    INFLATION_RADIUS_CELLS = 1
    inflated_grid = inflate_obstacles(occupancy_grid, INFLATION_RADIUS_CELLS)

    # Update plot only if interval has passed and plots are initialized
    # if fig is not None and (time.time() - last_plot_time > PLOT_INTERVAL):
    #     INFLATION_RADIUS_CELLS = 1
    #     inflated_grid = inflate_obstacles(occupancy_grid, INFLATION_RADIUS_CELLS)
        
    #     # img.set_data(occupancy_grid.T)
    #     # img.set_clim(vmin=0, vmax=2)
    #     # gx,gy=map_to_grid(x_d,y_d)
    #     # robot_dot.set_offsets((gx, gy)) 

    #     # img_inflated.set_data(inflated_grid.T)
    #     # img_inflated.set_clim(vmin=0, vmax=2)
    #     # robot_dot_inflated.set_offsets((gx, gy))
        
    #     plt.pause(0.02)
    #     last_plot_time = time.time()





    



async def publish_grid():
    """Publish occupancy grid to Redis and emit update event"""
    await redis.client.set("occupancy_grid", occupancy_grid.tobytes())
    await redis.publish("event:occupancy_grid_updated", {"status": "updated", "timestamp": time.time()})
    logger.info(f"[{WORKER_ID}] Occupancy grid published")


# ---------------- Redis Event Listeners ---------------- #

@redis.listen("mission_manager:drone_pose_update")
async def handle_drone_pose_update(data):
    """Update drone pose from Mission Manager"""
    global drone_pose
    drone_pose["x"] = data.get("position", [0, 0, 0])[0]
    drone_pose["y"] = data.get("position", [0, 0, 0])[1]
    logger.debug(f"[{WORKER_ID}] Pose updated: {drone_pose}")

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
    # print(" Angle offset:",angle_offset," Angle increment:",angle_inc,"drone pose:",drone_pose)
    update_grid(drone_pose["x"], drone_pose["y"], drone_pose["yaw"],
                distances, angle_offset, angle_inc)
    await publish_grid()


@redis.listen("mission_manager:system_time")
async def handle_system_time(data):
    """Update time sync"""
    global time_boot_us
    time_boot_us = data.get("time_boot_us", 0)


# ---------------- Worker Loops ---------------- #

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

    