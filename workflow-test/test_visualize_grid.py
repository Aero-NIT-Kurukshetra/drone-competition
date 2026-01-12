"""
Test 7: Visualize Occupancy Grid
Plots the occupancy grid with obstacles, crop locations, and drone positions.
"""
import asyncio
import json
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import asyncio

from dotenv import load_dotenv

load_dotenv()

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.redis_client import RedisClient
from workers.SETTINGS import (
    PATH_PLANNING_GRID_SIZE,
    PATH_PLANNING_MAP_SIZE,
    PATH_PLANNING_RESOLUTION,
    LAT0, LON0,
    EARTH_RADIUS
)
import math

# ============================================
# FARM POLYGON (from NIDAR.kml)
# ============================================
FARM_POLYGON = [
    {"lon": 76.81589155096442, "lat": 29.94965311567124},
    {"lon": 76.8162492257966,  "lat": 29.94939808518254},
    {"lon": 76.81664355584154, "lat": 29.94963310749574},
    {"lon": 76.81673785885653, "lat": 29.95054251583795},
    {"lon": 76.81637533963627, "lat": 29.95078200571026},
    {"lon": 76.81601151843628, "lat": 29.95060813092637},
    {"lon": 76.81589155096442, "lat": 29.94965311567124},  # Close polygon
]


def gps_to_grid(lat, lon):
    """Convert GPS to grid coordinates."""
    dlat = math.radians(lat - LAT0)
    dlon = math.radians(lon - LON0)
    
    x = dlon * math.cos(math.radians(LAT0)) * EARTH_RADIUS
    y = dlat * EARTH_RADIUS
    
    gx = int((x + PATH_PLANNING_MAP_SIZE / 2) / PATH_PLANNING_RESOLUTION)
    gy = int((y + PATH_PLANNING_MAP_SIZE / 2) / PATH_PLANNING_RESOLUTION)
    
    return gx, gy


def grid_to_gps(gx, gy):
    """Convert grid coordinates back to GPS."""
    x = gx * PATH_PLANNING_RESOLUTION - PATH_PLANNING_MAP_SIZE / 2
    y = gy * PATH_PLANNING_RESOLUTION - PATH_PLANNING_MAP_SIZE / 2
    
    lat0_rad = math.radians(LAT0)
    
    dlat = y / EARTH_RADIUS
    dlon = x / (EARTH_RADIUS * math.cos(lat0_rad))
    
    lat = LAT0 + math.degrees(dlat)
    lon = LON0 + math.degrees(dlon)
    
    return lat, lon


# Grid cell values
GRID_FREE = 0
GRID_OBSTACLE = 1
GRID_UNEXPLORED = 2


def create_sample_grid(grid_size: int) -> np.ndarray:
    """Create sample occupancy grid with explored/unexplored regions and obstacles."""
    # Start fully unexplored
    grid = np.full((grid_size, grid_size), GRID_UNEXPLORED, dtype=np.uint8)
    
    # Mark center area as explored (simulating scout flew through)
    center = grid_size // 2
    explore_radius = 60  # cells
    for dx in range(-explore_radius, explore_radius + 1):
        for dy in range(-explore_radius, explore_radius + 1):
            if dx*dx + dy*dy <= explore_radius*explore_radius:
                gx, gy = center + dx, center + dy
                if 0 <= gx < grid_size and 0 <= gy < grid_size:
                    grid[gy, gx] = GRID_FREE
    
    # Obstacle 1: Tree line on east side
    gx1, gy1 = gps_to_grid(29.95030, 76.81680)
    for dx in range(-3, 4):
        for dy in range(-10, 11):
            gx, gy = gx1 + dx, gy1 + dy
            if 0 <= gx < grid_size and 0 <= gy < grid_size:
                grid[gy, gx] = GRID_OBSTACLE
    
    # Obstacle 2: Structure in southwest
    gx2, gy2 = gps_to_grid(29.94955, 76.81570)
    for dx in range(-5, 6):
        for dy in range(-5, 6):
            gx, gy = gx2 + dx, gy2 + dy
            if 0 <= gx < grid_size and 0 <= gy < grid_size:
                grid[gy, gx] = GRID_OBSTACLE
    
    # Obstacle 3: Small obstacle in center
    gx3, gy3 = gps_to_grid(29.95000, 76.81630)
    for dx in range(-2, 3):
        for dy in range(-2, 3):
            gx, gy = gx3 + dx, gy3 + dy
            if 0 <= gx < grid_size and 0 <= gy < grid_size:
                grid[gy, gx] = GRID_OBSTACLE
    
    return grid


async def main():
    loop = asyncio.get_event_loop()
    redis = RedisClient(loop=loop, worker_id="test_visualize_grid")
    
    await redis.connect()
    print("✓ Connected to Redis")

    await runner(redis)

async def runner(redis):
    print("\n" + "=" * 70)
    print("OCCUPANCY GRID VISUALIZATION")
    print("=" * 70)
    
    print(f"\nGrid Configuration:")
    print(f"  Origin (LAT0, LON0): ({LAT0}, {LON0})")
    print(f"  Map size: {PATH_PLANNING_MAP_SIZE}m x {PATH_PLANNING_MAP_SIZE}m")
    print(f"  Resolution: {PATH_PLANNING_RESOLUTION}m/cell")
    print(f"  Grid size: {PATH_PLANNING_GRID_SIZE}x{PATH_PLANNING_GRID_SIZE}")
    
    # Try to load existing grid from Redis
    print("\n[1] Loading data from Redis...")
    
    grid_raw = await redis.binary_client.get("occupancy_grid")
    if grid_raw:
        grid = np.frombuffer(grid_raw, dtype=np.uint8).reshape(
            (PATH_PLANNING_GRID_SIZE, PATH_PLANNING_GRID_SIZE)
        )
        print(f"  ✓ Loaded occupancy grid from Redis")
    else:
        print("  ⚠ No grid in Redis, creating sample grid...")
        grid = create_sample_grid(PATH_PLANNING_GRID_SIZE)
        await redis.binary_client.set("occupancy_grid", grid.tobytes())
        print(f"  ✓ Created and stored sample grid")
    
    obstacle_count = np.sum(grid == 1)
    free_count = np.sum(grid == 0)
    print(f"  ✓ Obstacle cells: {obstacle_count}")
    print(f"  ✓ Free cells: {free_count}")
    
    # Load crop locations
    crops = []
    crop_raw = await redis.client.get("crop_locations")
    if crop_raw:
        crops = json.loads(crop_raw)
        print(f"  ✓ Loaded {len(crops)} crop locations")
    else:
        print("  ⚠ No crop locations in Redis")
    
    # Load sprayer waypoints
    sprayer_waypoints = []
    wp_raw = await redis.client.get("path_planner:sprayer_waypoints")
    if wp_raw:
        data = json.loads(wp_raw)
        sprayer_waypoints = data.get("waypoints", [])
        print(f"  ✓ Loaded {len(sprayer_waypoints)} sprayer waypoints")
    
    # Load drone positions
    sprayer_pos = None
    scout_pos = None
    state_raw = await redis.client.get("mission:state")
    if state_raw:
        state = json.loads(state_raw)
        sprayer_loc = state["drones"]["sprayer"].get("current_loc", {})
        if sprayer_loc.get("lat"):
            sprayer_pos = (sprayer_loc["lat"], sprayer_loc["lon"])
            print(f"  ✓ Sprayer position: ({sprayer_pos[0]:.6f}, {sprayer_pos[1]:.6f})")
        
        scout_loc = state["drones"]["scout"].get("current_loc", {})
        if scout_loc.get("lat"):
            scout_pos = (scout_loc["lat"], scout_loc["lon"])
            print(f"  ✓ Scout position: ({scout_pos[0]:.6f}, {scout_pos[1]:.6f})")
    
    await redis.close()
    
    # ============================================
    # PLOTTING
    # ============================================
    print("\n[2] Creating visualization...")
    
    # Count cell types
    free_count = np.sum(grid == GRID_FREE)
    obstacle_count = np.sum(grid == GRID_OBSTACLE)
    unexplored_count = np.sum(grid == GRID_UNEXPLORED)
    total = grid.size
    print(f"  Grid cells: {total} total")
    print(f"    - Free (explored): {free_count} ({100*free_count/total:.1f}%)")
    print(f"    - Obstacles: {obstacle_count} ({100*obstacle_count/total:.1f}%)")
    print(f"    - Unexplored: {unexplored_count} ({100*unexplored_count/total:.1f}%)")
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    fig.suptitle('NIDAR Occupancy Grid Visualization', fontsize=14, fontweight='bold')
    
    # ---- LEFT PLOT: Grid View ----
    ax1.set_title('Grid Coordinates View')
    
    # Create custom colormap: 0=white (free), 1=black (obstacle), 2=light gray (unexplored)
    from matplotlib.colors import ListedColormap
    cmap_grid = ListedColormap(['white', 'black', '#CCCCCC'])
    
    # Plot grid (transpose for correct orientation)
    grid_display = grid.T  # Transpose so X is horizontal
    ax1.imshow(grid_display, cmap=cmap_grid, vmin=0, vmax=2, origin='lower', alpha=0.8)
    
    # Plot farm polygon
    poly_grid = [gps_to_grid(p["lat"], p["lon"]) for p in FARM_POLYGON]
    poly_x = [p[0] for p in poly_grid]
    poly_y = [p[1] for p in poly_grid]
    ax1.plot(poly_x, poly_y, 'b-', linewidth=2, label='Farm Boundary')
    ax1.fill(poly_x, poly_y, alpha=0.1, color='blue')
    
    # Plot crops
    for i, crop in enumerate(crops):
        gx, gy = gps_to_grid(crop["lat"], crop["lon"])
        ax1.scatter(gx, gy, c='red', s=150, marker='*', zorder=5, 
                   label='Crops' if i == 0 else '')
        ax1.annotate(f'C{i+1}', (gx+2, gy+2), fontsize=9, color='darkred')
    
    # Plot sprayer waypoints as path
    if sprayer_waypoints:
        wp_grid = [gps_to_grid(wp["lat"], wp["lon"]) for wp in sprayer_waypoints]
        wp_x = [p[0] for p in wp_grid]
        wp_y = [p[1] for p in wp_grid]
        ax1.plot(wp_x, wp_y, 'g--', linewidth=1.5, alpha=0.7, label='Sprayer Path')
        ax1.scatter(wp_x, wp_y, c='green', s=30, marker='o', zorder=4)
    
    # Plot drone positions
    if sprayer_pos:
        gx, gy = gps_to_grid(sprayer_pos[0], sprayer_pos[1])
        ax1.scatter(gx, gy, c='orange', s=200, marker='^', zorder=6, label='Sprayer')
    
    if scout_pos:
        gx, gy = gps_to_grid(scout_pos[0], scout_pos[1])
        ax1.scatter(gx, gy, c='purple', s=200, marker='^', zorder=6, label='Scout')
    
    # Plot grid center (origin)
    center = PATH_PLANNING_GRID_SIZE // 2
    ax1.scatter(center, center, c='black', s=100, marker='+', zorder=3, label='Origin (LAT0,LON0)')
    
    # Add legend for cell types
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='white', edgecolor='black', label='Explored (Free)'),
        Patch(facecolor='black', edgecolor='black', label='Obstacle'),
        Patch(facecolor='#CCCCCC', edgecolor='black', label='Unexplored'),
    ]
    ax1.legend(handles=legend_elements + ax1.get_legend_handles_labels()[0], loc='upper right')
    
    ax1.set_xlabel('Grid X')
    ax1.set_ylabel('Grid Y')
    ax1.set_xlim(0, PATH_PLANNING_GRID_SIZE)
    ax1.set_ylim(0, PATH_PLANNING_GRID_SIZE)
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')
    
    # ---- RIGHT PLOT: GPS View ----
    ax2.set_title('GPS Coordinates View')
    
    # Calculate GPS bounds for grid
    lat_min, lon_min = grid_to_gps(0, 0)
    lat_max, lon_max = grid_to_gps(PATH_PLANNING_GRID_SIZE, PATH_PLANNING_GRID_SIZE)
    
    # Plot unexplored and obstacles in GPS space
    unexplored_lats, unexplored_lons = [], []
    obstacle_lats, obstacle_lons = [], []
    
    # Sample every Nth cell for performance
    sample_step = 4
    for gy in range(0, PATH_PLANNING_GRID_SIZE, sample_step):
        for gx in range(0, PATH_PLANNING_GRID_SIZE, sample_step):
            cell = grid[gy, gx]
            if cell == GRID_UNEXPLORED:
                lat, lon = grid_to_gps(gx, gy)
                unexplored_lats.append(lat)
                unexplored_lons.append(lon)
            elif cell == GRID_OBSTACLE:
                lat, lon = grid_to_gps(gx, gy)
                obstacle_lats.append(lat)
                obstacle_lons.append(lon)
    
    if unexplored_lats:
        ax2.scatter(unexplored_lons, unexplored_lats, c='lightgray', s=2, alpha=0.3, label='Unexplored')
    if obstacle_lats:
        ax2.scatter(obstacle_lons, obstacle_lats, c='black', s=4, alpha=0.7, label='Obstacles')
    
    # Plot farm polygon
    poly_lons = [p["lon"] for p in FARM_POLYGON]
    poly_lats = [p["lat"] for p in FARM_POLYGON]
    ax2.plot(poly_lons, poly_lats, 'b-', linewidth=2, label='Farm Boundary')
    ax2.fill(poly_lons, poly_lats, alpha=0.1, color='blue')
    
    # Plot crops
    for i, crop in enumerate(crops):
        ax2.scatter(crop["lon"], crop["lat"], c='red', s=150, marker='*', zorder=5,
                   label='Crops' if i == 0 else '')
        ax2.annotate(f'C{i+1}', (crop["lon"]+0.00005, crop["lat"]+0.00005), 
                    fontsize=9, color='darkred')
    
    # Plot sprayer waypoints
    if sprayer_waypoints:
        wp_lons = [wp["lon"] for wp in sprayer_waypoints]
        wp_lats = [wp["lat"] for wp in sprayer_waypoints]
        ax2.plot(wp_lons, wp_lats, 'g--', linewidth=1.5, alpha=0.7, label='Sprayer Path')
        ax2.scatter(wp_lons, wp_lats, c='green', s=30, marker='o', zorder=4)
    
    # Plot drone positions
    if sprayer_pos:
        ax2.scatter(sprayer_pos[1], sprayer_pos[0], c='orange', s=200, marker='^', 
                   zorder=6, label='Sprayer')
    
    if scout_pos:
        ax2.scatter(scout_pos[1], scout_pos[0], c='purple', s=200, marker='^', 
                   zorder=6, label='Scout')
    
    # Plot origin
    ax2.scatter(LON0, LAT0, c='black', s=100, marker='+', zorder=3, label='Origin')
    
    ax2.set_xlabel('Longitude')
    ax2.set_ylabel('Latitude')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')
    
    # Zoom to farm area with some margin
    margin = 0.0003
    ax2.set_xlim(min(poly_lons) - margin, max(poly_lons) + margin)
    ax2.set_ylim(min(poly_lats) - margin, max(poly_lats) + margin)
    
    plt.tight_layout()
    
    # Save figure
    output_path = os.path.join(os.path.dirname(__file__), 'grid_visualization.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"  ✓ Saved to: {output_path}")
    
    print("\n[3] Displaying plot...")
    plt.show()
    
    print("\n" + "=" * 70)
    print("Done!")

    await asyncio.sleep(2)
    await runner(redis)


if __name__ == "__main__":
    asyncio.run(main())
