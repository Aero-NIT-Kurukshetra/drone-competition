import numpy as np

def angular_downsample(points, step_deg=4):
    #step_deg: angular resolution in degrees
    buckets = {}
    for x, y in points:
        angle = np.arctan2(y, x) # Calculate the angle in radians
        #key is the bucket index for this angle
        key = int(np.degrees(angle) // step_deg) # // is floor division example: 7.9 // 4 = 1.0

        dist = np.hypot(x, y) # Euclidean distance from origin
        if key not in buckets or dist < buckets[key][1]:
            buckets[key] = ((x, y), dist)

    return [v[0] for v in buckets.values()]
