# NIDAR Workflow Testing Guide

This folder contains test scripts to verify the complete scout and sprayer drone workflows.

## Prerequisites

1. **Redis Server** - Must be running on localhost:6379
2. **SITL Instances** - Both scout and sprayer SITL simulators
3. **Python Environment** - With all dependencies installed

## Setup

### 1. Start Redis

```bash
redis-server
```

### 2. Start SITL Simulators

**Terminal 1 - Scout SITL:**

```bash
cd ArduPilot
sim_vehicle.py -v ArduCopter -I 0 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:13550
```

**Terminal 2 - Sprayer SITL:**

```bash
cd ArduPilot
sim_vehicle.py -v ArduCopter -I 1 --out=udp:127.0.0.1:14560 --out=udp:127.0.0.1:13560
```

### 3. Start Workers

**Terminal 3 - Mission Manager:**

```bash
cd d:\Coding\NIDAR
python -m workers.mission_manager.main
```

**Terminal 4 - Path Planner:**

```bash
cd d:\Coding\NIDAR
python -m workers.path_planning.main
```

**Terminal 5 - LiDAR Processing (optional):**

```bash
cd d:\Coding\NIDAR
python -m workers.lidar_processing.main
```

## Test Scenarios

### Test 1: Scout Lawnmower Pattern Generation

Tests KML parsing and waypoint generation without flying.

**Drone Behavior:** ⚪ **No drone movement** - Pure data generation test

```bash
python workflow-test/test_scout_waypoints.py
```

**Expected Output:**

-   Generates lawnmower waypoints from KML
-   Stores waypoints in Redis
-   Prints waypoint list

---

### Test 2: Full Scout Mission

Tests complete scout workflow with SITL.

**Drone Behavior:** 🚁 **Scout ARMS, TAKES OFF, FLIES lawnmower pattern, LANDS**

```bash
python workflow-test/test_scout_mission.py
```

**Expected Output:**

-   Scout arms and takes off to 5m
-   Navigates through lawnmower pattern
-   Lands after completing all waypoints

---

### Test 3: Crop Detection Simulation

Simulates crop detection events to trigger sprayer.

**Drone Behavior:** ⚪ **No drone movement** - Pure event simulation test

```bash
python workflow-test/test_crop_detection.py
```

**Expected Output:**

-   Publishes crop_detected events
-   Verifies crops stored in Redis
-   Triggers sprayer planning if sprayer is idle

---

### Test 4: Sprayer Path Planning (A\*)

Tests A\* path planning to crop locations.

**Drone Behavior:** ⚪ **No drone movement** - Pure path planning algorithm test

```bash
python workflow-test/test_sprayer_planning.py
```

**Expected Output:**

-   Creates mock occupancy grid
-   Plans path from sprayer position to crop
-   Stores sprayer waypoints in Redis

---

### Test 5: Full Sprayer Workflow

Tests complete sprayer workflow including spray sequence.

**Drone Behavior:** 🚁 **Sprayer ARMS, TAKES OFF, FLIES to crop, HOVERS, DESCENDS, SPRAYS, ASCENDS**

```bash
python workflow-test/test_sprayer_mission.py
```

**Expected Output:**

-   Sprayer arms and takes off
-   Navigates to crop location
-   Hovers for center correction (1s)
-   Descends to spray altitude (1m)
-   Sprays for 5s
-   Ascends back to cruise altitude
-   Proceeds to next crop or idles

---

### Test 6: Full Integration Test

Runs both scout and sprayer workflows simultaneously.

**Drone Behavior:** 🚁🚁 **BOTH drones ARM, TAKE OFF, FLY concurrently** - Scout scans, Sprayer sprays

```bash
python workflow-test/test_full_integration.py
```

**Expected Output:**

-   Scout starts lawnmower pattern
-   Scout detects crops during flight
-   Sprayer responds to crop detections
-   Both drones operate concurrently

---

## Monitoring

### Watch Redis Events

```bash
python workflow-test/monitor_events.py
```

This will display all Redis pub/sub events in real-time.

### Check Redis State

```bash
python workflow-test/check_state.py
```

Displays current mission state, waypoints, and crop locations.

---

## Troubleshooting

### Common Issues

1. **"No waypoints available"**

    - Run `test_scout_waypoints.py` first to generate waypoints

2. **"Sprayer not responding"**

    - Check sprayer SITL is running
    - Verify Mission Manager connected to sprayer

3. **"Connection refused"**
    - Ensure Redis is running
    - Check SITL UDP ports match configuration

### Environment Variables

Create a `.env` file in project root:

```
REDIS_HOST=localhost
REDIS_PORT=6379
SCOUT_MAVLINK_UDP=udp:localhost:13550
SPRAYER_MAVLINK_UDP=udp:localhost:13560
```

---

## Additional Test Scenarios

### Test 8: LiDAR Mock Data

Tests LiDAR obstacle detection and occupancy grid updates.

**Drone Behavior:** ⚪ **No drone movement** - Simulates LiDAR sensor data only

```bash
python workflow-test/test_lidar_mock.py
```

**Expected Output:**

-   Simulates OBSTACLE_DISTANCE messages at various angles
-   Verifies grid cells are marked as obstacles
-   Tests cone sweeping pattern (±22.5° arc)
-   Validates grid persistence in Redis

---

### Test 9: Sprayer State Machine

Tests complete sprayer spray sequence state transitions.

**Drone Behavior:** 🚁 **Sprayer ARMS, TAKES OFF, executes full spray state sequence**

```bash
python workflow-test/test_spray_states.py
```

**Expected Output:**

-   Tests state flow: IDLE → NAVIGATING → HOVERING → DESCENDING → SPRAYING → ASCENDING → IDLE
-   Verifies altitude changes (cruise 5m → spray 2m)
-   Tests hover wait time (1s)
-   Tests spray duration (2s)
-   Validates state machine handles all transitions correctly

---

### Test 10: Mission Completion & RTL

Tests Return-To-Launch behavior when missions complete.

**Drone Behavior:** 🚁🚁 **BOTH drones ARM, TAKE OFF, complete missions, RETURN TO LAUNCH**

```bash
python workflow-test/test_mission_complete.py
```

**Expected Output:**

-   Tests scout RTL after completing all lawnmower waypoints
-   Tests sprayer RTL trigger when scout lands (EXTENDED_SYS_STATE)
-   Tests sprayer RTL after completing all crop sprays
-   Verifies MAV_CMD_NAV_RETURN_TO_LAUNCH commands are sent

---

### Test 11: Exploration Wait Scenario

Tests sprayer waiting for scout to explore path to crop.

**Drone Behavior:** 🚁 **Sprayer WAITS (no movement), Scout simulated movement updates grid**

```bash
python workflow-test/test_exploration_wait.py
```

**Expected Output:**

-   Creates partially explored grid (fog of war)
-   Crop detected in unexplored area
-   Sprayer enters WAITING_FOR_EXPLORATION mode
-   Scout explores toward crop
-   Sprayer successfully plans path after area explored
-   Validates exploration mechanics work correctly
