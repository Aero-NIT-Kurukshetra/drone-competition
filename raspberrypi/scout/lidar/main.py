from conversion import gps_to_local, polar_to_cartesian, lidar_to_map

import numpy as np
import matplotlib.pyplot as plt

from filtering import range_filter,std_filter,point_cloud_average
from downsampling import angular_downsample
from conversion import polar_to_cartesian   

import redis
import time
import json
import random

from rplidar import RPLidar


# Connect to Redis
r = redis.Redis(host='localhost', port=6379, decode_responses=True)
x_d= 0.0
y_d= 0.0
yaw= 0.0

PORT_NAME='COM17'       # <-- YOUR COM PORT
BAUDRATE=1000000        # RPLIDAR S2 baudrate
MAX_BUFFER_SIZE=5000


lidar =RPLidar(PORT_NAME,baudrate=BAUDRATE)
drone_pose=(x_d,y_d,yaw)

try :
    # print("LiDAR Info:")
    # print(lidar.get_info())
    # print("LiDAR Health:")
    # print(lidar.get_health())
    for scan in lidar.iter_scans(max_buf_meas=MAX_BUFFER_SIZE):
        # # points are comming in as (quality, angle_deg, distance_mm)    
        print("raw scan",len(scan))
        filtered_scan=range_filter(scan,min_range=10,max_range=15000)
        print("new",len(filtered_scan))
        points=[]
        for point in filtered_scan: 
        
            x, y = polar_to_cartesian(point[1], point[2])
          
            points.append((x, y))

        # filtered_std=std_filter(points,k=1.5)
        # p2=point_cloud_average(filtered_std, window=1)
        # plot_points(p2)
        downsampled = angular_downsample(points, step_deg=1.0)
        message = {
            "timestamp": time.time(),
            "drone_pose": drone_pose,
            "points": points
        }

        print("filtered points",len(downsampled))
        r.publish("lidar:scan", json.dumps(message))

        # print("Published:", message)
        time.sleep(0.02)  # 5 Hz


except Exception as e:
    print("Error connecting to LiDAR:", e)
    exit(1)
finally:
    lidar.stop()
    # lidar.disconnect()
