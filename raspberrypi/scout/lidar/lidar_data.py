from rplidar import RPLidar
import matplotlib.pyplot as plt
from conversion import polar_to_cartesian

PORT_NAME = 'COM17'
BAUDRATE = 1000000

lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE)

# ---------- Matplotlib setup ----------
plt.ion()  # interactive mode ON
fig, ax = plt.subplots(figsize=(6, 6))

scatter = ax.scatter([], [], s=5)
lidar_dot = ax.scatter(0, 0, c='red', label="LiDAR")

ax.set_aspect('equal')
ax.grid()
ax.legend()
MAX_RANGE_M = 2.5  # for axis limits
# ---------- Plot function ----------
def plot_scan(scan):
    xs, ys = [], []

    for _, angle, distance in scan:
        if distance > 0:
            x, y = polar_to_cartesian(angle, distance)
            xs.append(x)
            ys.append(y)

    
    ax.cla()  # 🔴 clear old scan

    ax.scatter(xs, ys, s=5)
    ax.scatter(0, 0, c='red', label="LiDAR")
       # 🔒 LOCK AXIS LIMITS (this fixes size change)
    ax.set_xlim(-MAX_RANGE_M, MAX_RANGE_M)
    ax.set_ylim(-MAX_RANGE_M, MAX_RANGE_M)
    ax.set_aspect('equal')
    ax.grid()
    ax.legend()

    plt.draw()
    plt.pause(0.001)  # allow UI refresh


try:
    print("LiDAR Info:", lidar.get_info())
    print("LiDAR Health:", lidar.get_health())

    for scan in lidar.iter_scans(max_buf_meas=5000):
        plot_scan(scan)
        print("Total points:", len(scan))

except KeyboardInterrupt:
    print("Stopping LiDAR...")

finally:
    lidar.stop()
    lidar.disconnect()
    plt.ioff()
    plt.close()
