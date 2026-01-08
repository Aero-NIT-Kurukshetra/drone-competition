import math
import random

# LiDAR parameters (similar to RPLIDAR S2L)
MAX_RANGE_MM = 18000   # 18 meters
MIN_RANGE_MM = 50
ANGLE_RES_DEG = 1      # 1 degree resolution (simple for now)

# Example obstacle (circle)
# obstacle at (5, 3) meters with radius 0.5 m
OBSTACLES = [
      {"x": 5.0, "y": 3.0, "r": 0.5},
    {"x": 11.0, "y": 3.0, "r": 0.5},
    {"x": 23.0, "y":11.0, "r": 0.5},
    {"x": 5.0, "y": 7.0, "r": 0.5}
]

# def simulate_lidar_scan():
#     """
#     Simulates ONE full 360° LiDAR scan
#     Returns a list of points (angle, distance)
#     """
#     scan = []
#     new_scan = True

#     for angle in range(0, 360, ANGLE_RES_DEG):
#         distance_mm = MAX_RANGE_MM
#         theta = math.radians(angle)

#         # Ray casting (simple)
#         for d_mm in range(100, MAX_RANGE_MM, 50):
#             x = (d_mm / 1000.0) * math.cos(theta)
#             y = (d_mm / 1000.0) * math.sin(theta)
          
#             for obs in OBSTACLES:
#                 if (x - obs["x"])**2 + (y - obs["y"])**2 <= obs["r"]**2:
#                     distance_mm = d_mm
#                     break

#             if distance_mm != MAX_RANGE_MM:
#                 break

#         # Add small noise
#         distance_mm += random.randint(-20, 20)
#         distance_mm = max(MIN_RANGE_MM, min(distance_mm, MAX_RANGE_MM))

#         scan.append({
#             "angle_deg": angle,
#             "distance_mm": distance_mm,
#             "new_scan": new_scan
#         })

#         new_scan = False

#     return scan

def simulate_lidar_scan(x_d, y_d):
    """
    Simulates one 360° LiDAR scan.
    Returns list of dicts with angle & distance.
    """

    scan = []
    new_scan = True

    for angle in range(0, 360, ANGLE_RES_DEG):
        theta = math.radians(angle)
        distance_mm = MAX_RANGE_MM

        # Step size in meters (coarse → fast)
        step_m = 0.1

        # Ray march
        for d in range(1, int(MAX_RANGE_MM / 100)):
            dist_m = d * step_m
            x = x_d + dist_m * math.cos(theta)
            y = y_d + dist_m * math.sin(theta)

            for obs in OBSTACLES:
                dx = x - obs["x"]
                dy = y - obs["y"]

                if dx*dx + dy*dy <= obs["r"]**2:
                    distance_mm = int(dist_m * 1000)
                    break

            if distance_mm != MAX_RANGE_MM:
                break

        # Add realistic noise
        distance_mm += random.randint(-15, 15)
        distance_mm = max(MIN_RANGE_MM, min(distance_mm, MAX_RANGE_MM))

        scan.append({
            "angle_deg": angle,
            "distance_mm": distance_mm,
            "new_scan": new_scan
        })

        new_scan = False

    return scan