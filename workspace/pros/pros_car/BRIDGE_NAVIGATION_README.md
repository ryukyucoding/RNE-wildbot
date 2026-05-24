# Bridge Navigation Mission — Complete Guide

> **Robot:** RNE Wildbot (differential-drive) · **Stack:** ROS 2 Humble · **Sim:** Unity · **Viz:** Foxglove  
> **Branch:** `feat/bridge-navigation`

---

## Overview

The mission drives the robot autonomously to a bridge, aligns with it, and crosses it in three timed segments.

```
Phase 1 ─ Scan the map  ──────────────────────────────────────────────────────►
           (slam_toolbox builds an occupancy grid while you drive manually)
           ↓ save map  ↓ stop SLAM

Phase 2 ─ Fill in coordinates ───────────────────────────────────────────────►
           (read Point A & B off Foxglove, compute heading, edit YAML)

Phase 3 ─ Run the mission ───────────────────────────────────────────────────►
           APPROACH (Nav2) → ALIGN (rotate) → CROSS UP → PLATFORM → CROSS DOWN
```

### Why two regimes?

The map is a flat 2D occupancy grid. The bridge ramp appears as an **obstacle** and AMCL
localization degrades when the robot pitches on the incline (the 2D scan plane tilts).
Nav2 therefore **cannot cross the bridge** — it only plans on flat ground.

| Regime | Ground truth | Who controls |
|--------|-------------|--------------|
| Approach + Align | AMCL pose (flat ground) | Nav2 + action server |
| Cross Up / Platform / Cross Down | **None** — open-loop timed | Action server only |

No IMU is available or assumed. The crossing is dead-reckoned through three timed segments;
durations must be tuned on the real bridge.

---

## Directory Layout

```
workspace/pros/
├── pros_app/
│   ├── control.py                         # interactive menu launcher
│   ├── mission/
│   │   ├── start_slam.sh                  # Phase 1: bring up SLAM stack
│   │   ├── save_map.sh                    # Phase 1 end: save + stop SLAM
│   │   ├── start_nav.sh                   # Phase 3: bring up Nav stack
│   │   ├── set_pose.sh                    # helper: auto-set AMCL initial pose
│   │   └── stop_all.sh                    # emergency: kill all containers
│   └── docker/compose/
│       ├── docker-compose_rosbridge_server.yml
│       ├── docker-compose_robot_unity.yml
│       ├── docker-compose_slam_unity.yml
│       ├── docker-compose_localization_unity.yml
│       └── docker-compose_navigation_unity.yml
└── pros_car/
    ├── car_control.sh                     # launches pros_car Docker container
    └── src/car_control_pkg/
        ├── car_control_pkg/
        │   ├── car_control_common.py      # BaseCarControlNode + bridge params
        │   ├── car_nav_controller.py      # state machine: bridge_nav()
        │   └── car_action_server.py       # routes "Bridge_Nav" mode
        └── launch/
            ├── bridge_params.yaml         ← YOU EDIT THIS in Phase 2
            └── bridge_nav.launch.py       # loads bridge_params.yaml
```

---

## Phase 1 — Scan the Map

### What this phase does
Starts slam_toolbox and lets you drive the robot to build a 2D occupancy grid of the
bridge environment. When finished, saves the map to disk and stops SLAM.

> ⚠️  **SLAM and AMCL cannot run at the same time.**  
> SLAM publishes the `map → odom` TF and continuously rebuilds the map from scratch.  
> AMCL also publishes `map → odom` and needs a *static* saved map. Running both causes  
> TF conflicts and garbled robot position. Always stop SLAM before starting AMCL.

---

### Terminal 1 — Start the SLAM stack

```bash
cd ~/Desktop/114-2/RNE/Final_Project/RNE-wildbot/workspace/pros/pros_app

# Option A — using the mission script (recommended)
./mission/start_slam.sh

# Option B — using the interactive menu
python3 ./control.py -s
# Enter: 13 → b      (rosbridge_server — keeps running in background)
# Enter: 2  → b      (SLAM Unity       — keeps running in background)
```

**What starts:**
| Container | Purpose |
|-----------|---------|
| `compose-rosbridge-1` | Foxglove websocket bridge (port 8765) |
| `compose-robot_bringup-1` | LiDAR, odometry, TF tree |
| `compose-slam-1` | slam_toolbox — builds `/map` topic |

**Then:**
1. Open **Unity** → press **Play**
2. Open **Foxglove** → Connect → `ws://localhost:8765`
3. Add **3D Panel** → click **"+"** → add topic `/map`
4. You should see the occupancy grid grow as you drive

---

### Terminal 2 — Drive the robot

```bash
cd ~/Desktop/114-2/RNE/Final_Project/RNE-wildbot/workspace/pros/pros_car

./car_control.sh          # enter the pros_car Docker container
```

Inside the container:
```bash
r                                          # rebuild + source workspace
ros2 run pros_car_py robot_control         # start keyboard teleop
```

**Keyboard controls:**

| Key | Action |
|-----|--------|
| `w` | Forward |
| `s` | Backward |
| `a` | Turn left |
| `d` | Turn right |
| `e` | Rotate CCW (spin in place) |
| `r` | Rotate CW  (spin in place) |
| `q` | Stop / quit |

**Driving goal:** cover the entire bridge area — up the ramp, across the platform, down the
far side, and the surrounding flat ground. The more complete the scan, the better Nav2 can
plan an obstacle-free approach route.

Watch `/map` in Foxglove — grey = unknown, white = free, black = obstacle (walls + bridge sides).

---

### Read bridge coordinates (still in Terminal 2)

While still inside the container, echo the clicked_point topic:

```bash
ros2 topic echo /clicked_point
```

Leave this running. Switch to Foxglove.

**In Foxglove 3D Panel:**
1. In the toolbar at the top of the 3D panel, find **"Publish point"** (crosshair/target icon)
2. Click it to activate the tool
3. **Click Point A** — flat ground on the bridge centerline, *just before the up-ramp begins*  
   (≈ 0.3 m in front of where the incline starts — must be reachable by Nav2 on flat ground)
4. Check Terminal 2 — it prints:
   ```
   header:
     frame_id: map
   point:
     x: 1.234
     y: -0.987
     z: 0.0
   ```
   **Write down A.x and A.y.**

5. **Click Point B** — flat ground on the centerline, *just after the down-ramp ends*  
   **Write down B.x and B.y.**

6. Press `Ctrl+C` to stop the echo

**Compute heading:**
```bash
python3 -c "
import math
Ax = 1.234   # ← your Point A x
Ay = -0.987  # ← your Point A y
Bx = 2.456   # ← your Point B x
By = 0.123   # ← your Point B y
heading = math.degrees(math.atan2(By - Ay, Bx - Ax))
print(f'bridge_heading_deg = {heading:.2f}')
"
```

> **Tip:** optionally click a third point at the top of the platform to estimate where the
> up-ramp ends. Use the distance `hypot(platform − foot)` to seed `cross_up_sec` and
> `hypot(B − platform)` for `cross_down_sec`.

---

### Terminal 1 — Save the map and stop SLAM

```bash
# Option A — mission script
./mission/save_map.sh

# Option B — control.py menu
# Enter: 5       (store map — wait for "map saved" confirmation)
# Enter: d       (shut down all services)
```

The map is saved to `docker/compose/demo/map/mapXX/`. SLAM containers stop.  
**Do not start SLAM again** until you want a completely fresh map.

---

## Phase 2 — Fill in the Coordinates

Open the parameters file in VS Code:

```
workspace/pros/pros_car/src/car_control_pkg/launch/bridge_params.yaml
```

Edit the values with what you just measured:

```yaml
car_control_node:
  ros__parameters:

    # ── Approach goal (Point A) ─────────────────────────────────────────
    bridge_foot_x: 1.234          # Point A  x  (from /clicked_point)
    bridge_foot_y: -0.987         # Point A  y
    bridge_foot_yaw: 35.0         # same as bridge_heading_deg
    bridge_heading_deg: 35.0      # result of atan2(B−A) in degrees

    # ── Tolerances ──────────────────────────────────────────────────────
    foot_reached_thresh_m: 0.3    # stop Nav2 when within 0.3 m of foot
    align_tol_deg: 8.0            # acceptable heading error before climbing

    # ── Crossing durations (seconds) — tune on the real bridge ──────────
    cross_up_sec: 3.0             # time to climb the up-ramp
    cross_platform_sec: 2.0       # time to cross the flat platform
    cross_down_sec: 3.0           # time to descend the far ramp

    # ── Speed per segment ───────────────────────────────────────────────
    cross_up_action: "FORWARD"         # full power up the ramp
    cross_platform_action: "FORWARD_SLOW"
    cross_down_action: "FORWARD_SLOW"  # controlled descent
```

**Speed reference** (from `action_config.py`):

| Action name | Wheel speed |
|---|---|
| `FORWARD` | 10.0 (all four wheels) |
| `FORWARD_SLOW` | ~3.0 (all four wheels — exact value in `vel_slow`) |
| `CLOCKWISE_ROTATION` | ±10.0 (differential) |

> **All numeric values must have a decimal point** (e.g. `3.0` not `3`). ROS parameter
> loading treats bare integers differently from floats.

Save the file. No rebuild needed — the launch file reads this YAML at runtime.

---

## Phase 3 — Run the Mission

> **Recommended (Unity): SLAM stays running** — use `./mission/start_nav.sh` after Phase 1.
> See [`BRIDGE_NAVIGATION.md`](BRIDGE_NAVIGATION.md) for the full Chinese quick-start.
> An AMCL-based workflow (stop SLAM, use localization) is documented below as **legacy/alternative**.

### Terminal 1 — Start the navigation stack

```bash
cd ~/Desktop/114-2/RNE/Final_Project/RNE-wildbot/workspace/pros/pros_app

# Option A — mission script (recommended; SLAM continues from Phase 1)
./mission/start_nav.sh

# Option B — legacy AMCL workflow (stop SLAM first, then use control.py)
python3 ./control.py -s
# Enter: 13 → b     (rosbridge)
# Enter: 8  → b     (localization Unity: robot + AMCL + Nav2)
```

**What starts (Option A — SLAM continues):**
| Container | Purpose |
|-----------|---------|
| `compose-rosbridge-1` | Foxglove bridge (port 8765) |
| `compose-robot_bringup-1` | LiDAR, odometry, TF |
| SLAM container (still running) | Live map + `map→odom` TF |
| `compose-navigation-1` | Nav2 — costmap + planner, publishes `/received_global_plan` |

No AMCL initial pose needed — SLAM has tracked the robot since Phase 1.

**What starts (Option B — legacy AMCL):**
| Container | Purpose |
|-----------|---------|
| `compose-localization-1` | AMCL — loads saved map, publishes robot pose |
| `compose-navigation-1` | Nav2 — costmap + planner, publishes `/received_global_plan` |

**Then:**
1. Open **Unity** → press **Play**
2. Open **Foxglove** → Connect → `ws://localhost:8765`

---

### Set AMCL initial pose (legacy Option B only)

Skip this section if you used `./mission/start_nav.sh` (SLAM continues).

AMCL cannot localize until it knows roughly where the robot is. You have two options:

**Option A — Foxglove 2D Pose Estimate** *(most accurate)*
1. In the Foxglove 3D Panel toolbar, click **"2D Pose Estimate"** (arrow icon)
2. Find the robot's actual location on the map
3. **Click and hold** at that location
4. **Drag** in the direction the robot is facing (the arrow shows the heading)
5. **Release** — AMCL receives the estimate via `/initialpose`

Confirm it worked:
```bash
ros2 topic echo /amcl_pose --once
```
You should see a `PoseWithCovarianceStamped` message.

**Option B — set_pose.sh** *(automatic, reads /odom)*
```bash
# In a new terminal, exec into any running container:
docker exec -it compose-robot_bringup-1 bash

# Then inside the container, navigate to pros_app and run:
bash /path/to/pros_app/mission/set_pose.sh
```
This reads the current `/odom` position and publishes it to `/initialpose` automatically.
Works best when the robot hasn't moved far from the Unity spawn point (odom ≈ map origin).

**Option C — Unity spawn is always (0, 0)**  
If the robot hasn't moved since pressing Play, its map position is exactly `(0, 0, yaw=0)`:
```bash
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" --once
```

---

### Terminal 2 — Launch the car control node

> **Do NOT** use the `./car_control.sh` interactive menu or `robot_control` — those are not Bridge_Nav.

```bash
cd ~/Desktop/114-2/RNE/Final_Project/RNE-wildbot/workspace/pros/pros_car

./car_control.sh          # enter the pros_car Docker container
```

Inside the container:
```bash
r                                               # rebuild + source
ros2 launch car_control_pkg bridge_nav.launch.py
```

Wait until you see:
```
[car_control_node] Navigation Action Server initialized
```

This means the action server is ready and the `Bridge_Nav` goal handler is loaded.

> **Alternative — run with a custom params file:**
> ```bash
> ros2 launch car_control_pkg bridge_nav.launch.py \
>   params_file:=/path/to/your/bridge_params.yaml
> ```

---

### Terminal 3 — Trigger the mission

```bash
# Find the pros_car container name
docker ps
# Look for the row with image "pros_car_docker_image"
# The container name is in the last column (e.g. "epic_colden")

docker exec -it <container_name> bash
```

Inside the container:
```bash
source /workspaces/install/setup.bash

ros2 action send_goal /nav_action_server action_interface/action/NavGoal \
  "{mode: 'Bridge_Nav'}"
```

---

### Expected log sequence (Terminal 2)

```
Bridge_Nav reset; params={foot_x=..., foot_y=..., heading_deg=..., ...}
Published bridge foot goal to /goal_pose: (x, y, yaw=... deg)
APPROACH: following Nav2 plan...
Bridge_Nav: reached foot (dist=... m); aligning
ALIGN: diff=32.1° → rotating CW
ALIGN: diff=2.3° → close enough; starting climb
Bridge_Nav: CROSS_UP done -> CROSS_PLATFORM
Bridge_Nav: CROSS_PLATFORM done -> CROSS_DOWN
Bridge_Nav: CROSS_DOWN done -> DONE
Bridge_Nav: bridge traversed
```

If stuck in APPROACH you may see (every 2 s):
`APPROACH: waiting for global plan on /received_global_plan...`

---

### Emergency stop

| Method | Command |
|--------|---------|
| Cancel the action goal | `Ctrl+C` in Terminal 3 |
| Stop all containers | `./mission/stop_all.sh` from `pros_app/` |
| Kill just navigation | `docker compose -f docker/compose/docker-compose_navigation_unity.yml down` |

---

## Tuning the Crossing Durations

The three `cross_*_sec` values must be measured on the actual bridge geometry.
They are the **only** control knobs during the crossing — there is no sensor feedback.

### Step-by-step tuning procedure

1. **Place the robot at the foot**, already aligned with the bridge centerline (skip APPROACH).
2. Edit `bridge_params.yaml` — set `cross_up_sec` to a conservative value (start at `2.0`).
3. Trigger the mission (Terminal 3 command above).
4. Watch where the robot stops: if it stops *before* the platform, increase the value.
5. Repeat until the robot reliably reaches the top.
6. Repeat for `cross_platform_sec` and `cross_down_sec`.

### Tuning tips

| Symptom | Fix |
|---------|-----|
| Robot stalls on ramp | Keep `cross_up_action: "FORWARD"` (max power); check battery level |
| Robot veers sideways on ramp | Trim wheel speeds in `action_config.py`; or adjust physical alignment |
| Overshoots far end | Reduce `cross_down_sec` |
| AMCL jumps during crossing | Expected and harmless — AMCL is not used on the bridge |
| Robot doesn't stop at DONE | Verify `cancel_callback` is wired to `publish_control("STOP")` in action server |

### Unity vs Real Robot

| Factor | Unity sim | Real robot |
|--------|-----------|------------|
| Ramp modelled in 3D? | Depends on scene | Always yes |
| Duration values valid? | Only if ramp is 3D in scene | Must re-measure |
| Logic / state machine | Validates correctly | Same code |
| Use sim for | State machine + approach | Approach refinement only |
| Use real for | N/A | Tune `cross_*_sec` |

---

## State Machine Reference

```
reset_bridge() called ──► APPROACH
                              │
                              │  distance to foot < foot_reached_thresh_m
                              ▼
                           ALIGN
                              │
                              │  |heading error| < align_tol_deg
                              ▼
                          CROSS_UP ──► elapsed ≥ cross_up_sec ──► CROSS_PLATFORM
                                                                         │
                                                              elapsed ≥ cross_platform_sec
                                                                         ▼
                                                                    CROSS_DOWN
                                                                         │
                                                              elapsed ≥ cross_down_sec
                                                                         ▼
                                                                       DONE
                                                                  (return success)
```

### APPROACH
- Publishes the bridge foot as a `PoseStamped` to `/goal_pose` (once on entry)
- Nav2 picks it up, generates a global plan avoiding the bridge walls (costmap inflation)
- The action server follows `/received_global_plan` using `get_next_target_point` →
  `calculate_diff_angle` → `choose_action` → `publish_control`
- Transitions when `cal_distance(amcl_xy, foot_xy) < foot_reached_thresh_m`

### ALIGN
- Reads current yaw from TF (`map→base_footprint`) via `get_yaw_from_quaternion`
- Computes `diff = normalize(bridge_heading_deg − current_yaw)`
- `diff > 0` → `COUNTERCLOCKWISE_ROTATION`; `diff < 0` → `CLOCKWISE_ROTATION`
- Uses `_MEDIAN` variant when |diff| < 30°, `_SLOW` when |diff| < 15°
- Transitions when `|diff| < align_tol_deg`

### CROSS_UP / CROSS_PLATFORM / CROSS_DOWN
- Pure open-loop: calls `publish_control(action)` every tick (10 Hz)
- Transitions after `time.time() − bridge_seg_start >= duration`
- No sensor feedback; no correction for veering

### DONE
- Publishes `STOP` 5× to ensure motors halt
- Calls `clear_plan()` and `clear_goal_pose()`
- Returns `NavGoal.Result(success=True, message="bridge traversed")`

---

## Troubleshooting

### `/amcl_pose` not publishing

```bash
ros2 topic echo /amcl_pose --once
# "does not appear to be published yet" or no output
```

**Cause:** AMCL is running but has no initial pose estimate.  
**Fix:** Use any of the three options in "Set AMCL initial pose" above.

---

### `/received_global_plan` empty / Bridge_Nav stuck in APPROACH

```bash
ros2 topic echo /received_global_plan --once
# no output
```

**Cause 1:** Nav2 hasn't received a goal yet (normal for first few seconds — wait).  
**Cause 2:** `/goal_pose` was published before Nav2 finished starting.  
**Fix:** Wait ~10 s after `start_nav.sh`, then re-trigger the mission.

**Cause 3:** Nav2 cannot find a path — bridge foot is inside a costmap obstacle.  
**Fix:** Move `bridge_foot_x/y` slightly further from the bridge (increase clearance from ramp).

---

### Accidentally ran SLAM after localization

**Symptom:** `/map` resets to empty; robot position jumps to origin.  
**Fix:**
```bash
./mission/stop_all.sh   # kill everything
# then restart:
./mission/start_nav.sh
# redo the initial pose step
```

---

### SLAM was stopped — can't see robot in Foxglove

**Symptom:** Foxglove 3D panel shows the saved map but no robot marker.  
**Explanation:** The robot marker comes from AMCL, which needs an initial pose first.  
**Fix:** Use `set_pose.sh` or the `/initialpose` command (Option C above) — you do **not**  
need to see the robot to set the pose; just use (0, 0) if Unity just started.

---

### Can I keep SLAM running during navigation?

**Yes — this is the recommended Unity workflow** (`start_slam.sh` → `save_map.sh` → `start_nav.sh`).
SLAM continues to provide `map→odom→base_footprint` TF; Nav2 adds planning on top.

**Do not run SLAM and AMCL together** — they both publish `map→odom` and conflict.

Legacy real-robot workflow: **SLAM → save → stop SLAM → start AMCL/Nav2** (Option B above).

---

## Quick Reference Card

```
╔══════════════════════════════════════════════════════════════════╗
║  PHASE 1 — SCAN                                                  ║
║  Terminal 1: ./mission/start_slam.sh                             ║
║  Terminal 2: ./car_control.sh → r → ros2 run pros_car_py ...     ║
║              Drive around bridge. Read /clicked_point (A, B).   ║
║  Terminal 1: ./mission/save_map.sh                               ║
╠══════════════════════════════════════════════════════════════════╣
║  PHASE 2 — EDIT YAML                                             ║
║  Edit: launch/bridge_params.yaml                                 ║
║  Fill: bridge_foot_x/y, bridge_heading_deg, cross_*_sec          ║
╠══════════════════════════════════════════════════════════════════╣
║  PHASE 3 — MISSION                                               ║
║  Terminal 1: ./mission/start_nav.sh                              ║
║              Unity Play → Foxglove 2D Pose Estimate (if needed)  ║
║  Terminal 2: ./car_control.sh → r →                              ║
║              ros2 launch car_control_pkg bridge_nav.launch.py    ║
║  Terminal 3: docker exec -it <container> bash                    ║
║              ros2 action send_goal /nav_action_server \          ║
║                action_interface/action/NavGoal "{mode:'Bridge_Nav'}"
╠══════════════════════════════════════════════════════════════════╣
║  EMERGENCY STOP                                                  ║
║  Ctrl+C in Terminal 3  ─OR─  ./mission/stop_all.sh               ║
╚══════════════════════════════════════════════════════════════════╝
```

---

## Files Modified / Created

| File | Change |
|------|--------|
| `car_control_pkg/car_control_common.py` | Added `_declare_bridge_parameters()`, `get_bridge_params()`, `publish_goal_pose()` |
| `car_control_pkg/car_nav_controller.py` | Added `reset_bridge()`, `bridge_nav()` state machine |
| `car_control_pkg/car_action_server.py` | Added `Bridge_Nav` branch in `_select_car_auto_method` and `execute_callback` |
| `car_control_pkg/setup.py` | Added `launch/` to `data_files` |
| `launch/bridge_params.yaml` | **New** — all 12 bridge parameters |
| `launch/bridge_nav.launch.py` | **New** — launches node with params file |
| `pros_app/mission/start_slam.sh` | **New** — Phase 1 stack startup |
| `pros_app/mission/save_map.sh` | **New** — map save + SLAM shutdown |
| `pros_app/mission/start_nav.sh` | **New** — Phase 3 stack startup |
| `pros_app/mission/set_pose.sh` | **New** — auto-set AMCL initial pose from /odom |
| `pros_app/mission/stop_all.sh` | **New** — kill all mission containers |
