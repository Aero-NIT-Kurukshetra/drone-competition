import numpy as np

#range filter to remove points outside specified range
def range_filter(scan, min_range=200, max_range=15000):
    filtered=[]
    scan = [
    point for point in scan
    if min_range < point[2] < max_range

    ]
    return scan

# standard deviation filter to remove outlier points used mean+k*stddev
#but its very computationally expensive in O(n^2) for the raspberry pi

import numpy as np

def compute_neighbor_distances(points):
    distances = []

    for i, p in enumerate(points):
        local_distances = []
        for j, q in enumerate(points):
            if i != j:
                d = np.linalg.norm(np.array(p) - np.array(q))
                local_distances.append(d)

        # average distance to neighbors
        distances.append(np.mean(local_distances))

    return distances

def std_filter(points, k=1.5):
    if len(points) < 5:
        return points

    distances = compute_neighbor_distances(points)

    mean = np.mean(distances)
    std = np.std(distances)

    filtered = [
        p for p, d in zip(points, distances)
        if d < mean + k * std
    ]

    return filtered
# Or a less computationally expensive method like dcscan its wortscase O(n^2) but with early stopping
def filter_dcscan(points,radius=0.5,min_neighbors=3):
    filtered=[]
    for i, p in enumerate(points):
        neighbor_count=0
        for j,q in enumerate(points):
            if i==j:
                continue


            d =np.linalg.norm(np.array(p)-np.array(q))
            if d<radius:
                neighbor_count+=1
            if neighbor_count>=min_neighbors:
                filtered.append(p)
                break

    return filtered






def point_cloud_average(points, window=3):
    smoothed = []
    for i in range(len(points)):
        start = max(0, i - window)
        end = min(len(points), i + window + 1)
        avg_x = np.mean([p[0] for p in points[start:end]])
        avg_y = np.mean([p[1] for p in points[start:end]])
        smoothed.append((avg_x, avg_y))
    return smoothed

