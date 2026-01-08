

from matplotlib import pyplot as plt
from conversion import gps_to_local, polar_to_cartesian, lidar_to_map
from occupancy_grid import map_to_grid,create_occupancy_grid,update_grid_from_scan,inflate_obstacles

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

import redis
import json
grid=create_occupancy_grid()


MAX_LIMIT=10





# def plot_scan(points):
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




r =redis.Redis(host='localhost',port=6379,decode_responses=True)
pubsub=r.pubsub()
pubsub.subscribe("lidar:scan")
print("Listening for LiDAR data...")
scan_points=[]
scan_count=0


plt.ion()
fig, ax = plt.subplots(figsize=(8,8))
ax.set_xlim(0, grid.shape[0])
ax.set_ylim(0, grid.shape[1])
cmap = ListedColormap(["green", "red", "white"])

img = ax.imshow(grid.T, cmap=cmap, origin='lower', interpolation='nearest')
robot_dot, = ax.plot(0, 0, 'ro')
plt.show()
while True:
    message = next(pubsub.listen())
    # print(message)
    if message['type']=='message':
        data=json.loads(message['data'])
        points=data['points']  
        if(len(points)==0):
            continue
        

        drone_pos=data['drone_pose']
        timestamp=data['timestamp']
        print(f"Received LiDAR scan with {len(points)} drone_pos={drone_pos} timestamp={timestamp}  ")
    
       
        x_d, y_d, yaw= drone_pos
        points_map=[lidar_to_map(x,y,x_d,y_d,yaw) for (x,y) in points]

        # plot_scan(points_map)
        # continue
        print(drone_pos)
        update_grid_from_scan(points_map, grid, (x_d, y_d, yaw))
        INFLATION_RADIUS_CELLS = 5
        inflated_grid = inflate_obstacles(grid, INFLATION_RADIUS_CELLS)
        
        img.set_data(inflated_grid.T)
        img.set_clim(vmin=0, vmax=2)
        # gx,gy=map_to_grid(x_d,y_d)
        # robot_dot.set_data(gx, gy)

        # img.set_data(inflated_grid.T)
        plt.show()
        plt.pause(0.01)