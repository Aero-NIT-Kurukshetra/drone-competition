# Event Flow Documentation

This document describes all Redis pub/sub events in the NIDAR system, including which workers publish and subscribe to each event.

## Table of Contents

-   [Events Published by Worker](#events-published-by-worker)
    -   [Mission Manager Worker](#mission-manager-worker)
    -   [Path Planning Worker](#path-planning-worker)
    -   [LiDAR Processing Worker](#lidar-processing-worker)
    -   [Camera Processing Worker](#camera-processing-worker)
-   [Mission Control Events](#mission-control-events)
-   [Path Planning Events](#path-planning-events)
-   [LiDAR Processing Events](#lidar-processing-events)
-   [MAVLink Events](#mavlink-events)
-   [System Events](#system-events)
-   [Event Flow Diagram](#event-flow-diagram)

---

## Events Published by Worker

### Mission Manager Worker

Events published by [Mission Manager](workers/mission_manager/main.py) and [MAVLink Manager](workers/mission_manager/mavlink_manager.py):

| Event Name                                 | Receiver Workers                                                                                                          |
| ------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------- |
| `mission_manager:request_next_waypoint`    | [Path Planning Worker](workers/path_planning/main.py#L190)                                                                |
| `mission_manager:drone_pose_update`        | [Mission Manager](workers/mission_manager/main.py#L135), [LiDAR Processing Worker](workers/lidar_processing/main.py#L186) |
| `mission_manager:drone_attitude_update`    | [LiDAR Processing Worker](workers/lidar_processing/main.py#L194)                                                          |
| `mission_manager:lidar_obstacle_distance`  | [LiDAR Processing Worker](workers/lidar_processing/main.py#L202)                                                          |
| `mission_manager:drone_heartbeat`          | None (monitoring only)                                                                                                    |
| `mission_manager:drone_extended_sys_state` | None (monitoring only)                                                                                                    |

### Path Planning Worker

Events published by [Path Planning Worker](workers/path_planning/main.py):

| Event Name                       | Receiver Workers                                       |
| -------------------------------- | ------------------------------------------------------ |
| `path_planning:planned_waypoint` | [Mission Manager](workers/mission_manager/main.py#L75) |
| `event:pathplanner_out`          | None (error logging only)                              |

### LiDAR Processing Worker

Events published by [LiDAR Processing Worker](workers/lidar_processing/main.py):

| Event Name                                | Receiver Workers                                        |
| ----------------------------------------- | ------------------------------------------------------- |
| `lidar_processing:occupancy_grid_updated` | [Mission Manager](workers/mission_manager/main.py#L113) |

### Camera Processing Worker

Events published by [Camera Processing Worker](workers/camera_processing/main.py):

| Event Name                      | Receiver Workers                                        |
| ------------------------------- | ------------------------------------------------------- |
| `event:crop_detected` (planned) | [Mission Manager](workers/mission_manager/main.py#L123) |

> **Note**: Camera Processing Worker is currently a stub and does not publish any events yet.

---

## Mission Control Events

### `start_mission`

-   **Published by**: External trigger / Test scripts
-   **Received by**: [Mission Manager](workers/mission_manager/main.py#L49)
-   **Purpose**: Initiates the mission sequence
-   **Payload**: Empty object
-   **Handler**: `handle_start_mission()`
-   **Actions**:
    -   Resets halted state for all drones
    -   Sets drones to GUIDED mode
    -   Arms and takes off scout drone
    -   Publishes `mission_manager:request_next_waypoint` for scout

### `mission_manager:request_next_waypoint`

-   **Published by**: [Mission Manager](workers/mission_manager/main.py#L61)
-   **Received by**: [Path Planning Worker](workers/path_planning/main.py#L190)
-   **Purpose**: Requests the next waypoint from path planner
-   **Payload**:
    ```json
    {
      "drone_id": "scout" | "sprayer",
      "lat": float (optional),
      "lon": float (optional),
      "target": object (optional, for sprayer)
    }
    ```
-   **Handler**: `handle_request_next_waypoint()`

### `mission_manager:scout_planning_request`

-   **Published by**: External trigger / Test scripts
-   **Received by**: [Path Planning Worker](workers/path_planning/main.py#L118)
-   **Purpose**: Initiates path planning for scout drone with KML polygon
-   **Payload**:
    ```json
    {
    	"kml_xml": "<kml>...</kml>",
    	"spacing": 5,
    	"angle": 60
    }
    ```
-   **Handler**: `handle_pathplanner_event()`
-   **Actions**:
    -   Generates lawnmower pattern waypoints
    -   Stores waypoints in Redis key `path_planner:scout_waypoints`
    -   Stores current index in `path_planner:current_scout_waypoint_index`

---

## Path Planning Events

### `path_planning:planned_waypoint`

-   **Published by**: [Path Planning Worker](workers/path_planning/main.py#L230)
-   **Received by**: [Mission Manager](workers/mission_manager/main.py#L75)
-   **Purpose**: Sends a planned waypoint to mission manager for execution
-   **Payload**:
    ```json
    {
      "drone_id": "scout" | "sprayer",
      "waypoint": {
        "lat": float,
        "lon": float,
        "alt_m": float
      }
    }
    ```
-   **Handler**: `handle_planned_waypoint()`
-   **Actions**:
    -   Sends waypoint command via MAVLink
    -   Updates active waypoint state
    -   Resets halted flag

### `path_planning:arm_takeoff`

-   **Published by**: Path Planning Worker (not currently used in code)
-   **Received by**: [Mission Manager](workers/mission_manager/main.py#L66)
-   **Purpose**: Commands a drone to arm and takeoff
-   **Payload**:
    ```json
    {
      "drone_id": "scout" | "sprayer",
      "altitude": float
    }
    ```
-   **Handler**: `handle_arm_takeoff()`

### `event:pathplanner_out`

-   **Published by**: [Path Planning Worker](workers/path_planning/main.py#L162) (on error)
-   **Received by**: None (monitoring/logging only)
-   **Purpose**: Reports path planning errors
-   **Payload**:
    ```json
    {
      "worker_id": "path_planner_worker",
      "status": "error",
      "error": string,
      "timestamp": float
    }
    ```

---

## LiDAR Processing Events

### `lidar_processing:occupancy_grid_updated`

-   **Published by**: [LiDAR Processing Worker](workers/lidar_processing/main.py#L183)
-   **Received by**: [Mission Manager](workers/mission_manager/main.py#L113)
-   **Purpose**: Notifies that the occupancy grid has been updated with new LiDAR data
-   **Payload**:
    ```json
    {
      "status": "updated",
      "timestamp": float
    }
    ```
-   **Handler**: `handle_grid_update()`
-   **Actions**:
    -   Requests new waypoints for all non-halted drones
    -   Grid data stored in Redis key `occupancy_grid` as bytes

### `mission_manager:lidar_obstacle_distance`

-   **Published by**: [MAVLink Manager](workers/mission_manager/mavlink_manager.py#L181)
-   **Received by**: [LiDAR Processing Worker](workers/lidar_processing/main.py#L202)
-   **Purpose**: Raw LiDAR distance measurements from drone
-   **Payload**:
    ```json
    {
      "time_usec": int,
      "distances": [int],
      "angle_offset": float,
      "increment": float
    }
    ```
-   **Handler**: `handle_obstacle_distance()`
-   **Actions**:
    -   Filters and processes LiDAR points
    -   Updates occupancy grid
    -   Publishes `lidar_processing:occupancy_grid_updated`

---

## MAVLink Events

These events are published by the [MAVLink Manager](workers/mission_manager/mavlink_manager.py) based on incoming MAVLink messages from drones.

### `mission_manager:drone_pose_update`

-   **Published by**: [MAVLink Manager](workers/mission_manager/mavlink_manager.py#L157)
-   **Received by**:
    -   [Mission Manager](workers/mission_manager/main.py#L135)
    -   [LiDAR Processing Worker](workers/lidar_processing/main.py#L186)
-   **Purpose**: Updates drone's global position
-   **Payload**:
    ```json
    {
      "drone_id": "scout" | "sprayer",
      "lat": float,
      "lon": float,
      "alt_m": float,
      "timestamp": float
    }
    ```
-   **Handlers**:
    -   `handle_pose_update()` (Mission Manager) - Checks waypoint arrival
    -   `handle_drone_pose_update()` (LiDAR) - Updates drone position for grid mapping

### `mission_manager:drone_attitude_update`

-   **Published by**: [MAVLink Manager](workers/mission_manager/mavlink_manager.py#L166)
-   **Received by**: [LiDAR Processing Worker](workers/lidar_processing/main.py#L194)
-   **Purpose**: Updates drone's attitude (yaw)
-   **Payload**:
    ```json
    {
      "drone_id": "scout" | "sprayer",
      "yaw": float,
      "timestamp": float
    }
    ```
-   **Handler**: `handle_drone_yaw_update()`
-   **Actions**: Updates yaw for LiDAR point cloud transformation

### `mission_manager:drone_heartbeat`

-   **Published by**: [MAVLink Manager](workers/mission_manager/mavlink_manager.py#L127)
-   **Received by**: None (monitoring/logging only)
-   **Purpose**: Drone health and mode status
-   **Payload**:
    ```json
    {
      "drone_id": "scout" | "sprayer",
      "base_mode": int,
      "custom_mode": int,
      "system_status": int,
      "timestamp": float
    }
    ```

### `mission_manager:drone_extended_sys_state`

-   **Published by**: [MAVLink Manager](workers/mission_manager/mavlink_manager.py#L145)
-   **Received by**: None (monitoring/logging only)
-   **Purpose**: Extended system state (landed/in-air)
-   **Payload**:
    ```json
    {
      "drone_id": "scout" | "sprayer",
      "landed_state": int,
      "timestamp": float
    }
    ```

### `mission_manager:system_time`

-   **Published by**: MAVLink Manager (commented out)
-   **Received by**: [LiDAR Processing Worker](workers/lidar_processing/main.py#L219)
-   **Purpose**: Synchronizes system time with drone's boot time
-   **Payload**:
    ```json
    {
      "time_boot_us": int
    }
    ```
-   **Handler**: `handle_system_time()`

---

## System Events

### `system_mode`

-   **Published by**: External trigger / Supervisor
-   **Received by**: [Mission Manager](workers/mission_manager/main.py#L195)
-   **Purpose**: Changes system operation mode
-   **Payload**: `"NORMAL"` | `"RECOVERY"`
-   **Handler**: `handle_system_mode()`
-   **Actions**:
    -   In RECOVERY mode: halts all drones
    -   Updates mission state mode

### `event:no_safe_path`

-   **Published by**: Path Planning Worker (not currently implemented)
-   **Received by**: [Mission Manager](workers/mission_manager/main.py#L104)
-   **Purpose**: Indicates no safe path available for drone
-   **Payload**:
    ```json
    {
      "drone_id": "scout" | "sprayer"
    }
    ```
-   **Handler**: `handle_no_safe_path()`
-   **Actions**:
    -   Halts the specified drone
    -   Sets halted flag

### `event:crop_detected`

-   **Published by**: Camera Processing Worker (not implemented yet)
-   **Received by**: [Mission Manager](workers/mission_manager/main.py#L123)
-   **Purpose**: Notifies when crops are detected by camera
-   **Payload**:
    ```json
    {
      "location": {
        "lat": float,
        "lon": float
      }
    }
    ```
-   **Handler**: `handle_crop_detected()`
-   **Actions**:
    -   Requests sprayer waypoint to detected crop location

---

## Event Flow Diagram

### Mission Start Sequence

```
External Trigger
    │
    └─► start_mission
            │
            ▼
    Mission Manager
            │
            ├─► Sets GUIDED mode
            ├─► Arms & takeoff scout
            │
            └─► mission_manager:request_next_waypoint
                    │
                    ▼
            Path Planning Worker
                    │
                    └─► path_planning:planned_waypoint
                            │
                            ▼
                    Mission Manager
                            │
                            └─► Sends MAVLink command to drone
```

### LiDAR Processing Flow

```
Drone (MAVLink)
    │
    └─► OBSTACLE_DISTANCE
            │
            ▼
    MAVLink Manager
            │
            └─► mission_manager:lidar_obstacle_distance
                    │
                    ▼
            LiDAR Processing Worker
                    │
                    ├─► Processes point cloud
                    ├─► Updates occupancy grid
                    │
                    └─► lidar_processing:occupancy_grid_updated
                            │
                            ▼
                    Mission Manager
                            │
                            └─► mission_manager:request_next_waypoint
                                    │
                                    ▼
                            Path Planning Worker
                                    │
                                    └─► Replans path with new grid
```

### Waypoint Progress Flow

```
Drone (MAVLink)
    │
    └─► GLOBAL_POSITION_INT
            │
            ▼
    MAVLink Manager
            │
            └─► mission_manager:drone_pose_update
                    │
                    ├─► Mission Manager (checks arrival)
                    │       │
                    │       └─► mission_manager:request_next_waypoint
                    │               │
                    │               ▼
                    │       Path Planning Worker
                    │               │
                    │               └─► path_planning:planned_waypoint
                    │
                    └─► LiDAR Processing Worker (updates position)
```

---

## Notes

### Camera Processing Worker

The [Camera Processing Worker](workers/camera_processing/main.py) is currently a stub and does not publish or subscribe to any events yet. Future implementation should publish `event:crop_detected`.

### Boilerplate Worker

The [Boilerplate Worker](workers/boilerplate.py) is a template showing dummy events:

-   Listens to: `event:dummy_in`
-   Publishes: `event:dummy_out`

### Redis Keys

In addition to pub/sub events, the system uses Redis keys for state storage:

-   `occupancy_grid`: Binary occupancy grid data (bytes)
-   `path_planner:scout_waypoints`: JSON waypoint list
-   `path_planner:current_scout_waypoint_index`: Current waypoint index (string)
-   `path_planner:sprayer_waypoints`: JSON waypoint list (planned)
-   `path_planner:current_sprayer_waypoint_index`: Current waypoint index (planned)
