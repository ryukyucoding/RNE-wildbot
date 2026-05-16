# kros_car

ROS 2 Jazzy package for the KROS Car — a 4WD differential-drive mobile robot with a 3-DOF arm and gripper. Uses `ros2_control` with direct serial I/O in a single C++ hardware plugin.

## Architecture

```
cmd_vel / MoveIt / JointTrajectory action
        |
  ros2_control controllers (15 Hz)
  - base_controller       (diff_drive_controller)
  - arm_controller        (joint_trajectory_controller)
  - joint_state_broadcaster
        |
  KrosCarSystem           (C++ hardware plugin)
  |             |
Wheel motors    Arm servos
RS-485 serial   TTL serial
```

The hardware plugin talks to serial devices directly via **libserial** — no intermediate Python nodes or ROS topics. Three background threads handle asynchronous serial I/O:

- **Wheel TX thread** (~12.5 Hz effective) — sends JSON motor commands with 20 ms gap between motors
- **Arm TX thread** — sends latest arm servo targets from a shared command buffer
- **Arm poll thread** (~4 Hz per servo with default timeout) — reads servo positions and monitors gripper temperature for overheat protection

## Hardware

| Component | Hardware | Protocol | Default Port |
|---|---|---|---|
| Wheels (x4) | Hub motors | JSON over RS-485 | `/dev/usb_wheel` |
| Arm (x2) + Gripper (x1) | Bus servos | Binary packet over TTL serial | `/dev/usb_robot_arm` |

- Wheel radius: 50.35 mm, track width: 500 mm, max RPM: 100
- Servo range: 0-1000 ticks = 0-240 degrees (0-4.189 rad)

## Topics

### ros2_control (primary interface)

These are managed by the controllers and are the recommended way to command the robot.

| Topic | Type | Dir | Description |
|---|---|---|---|
| `/base_controller/cmd_vel` | `geometry_msgs/msg/TwistStamped` | sub | Drive command (linear.x + angular.z) |
| `/base_controller/odom` | `nav_msgs/msg/Odometry` | pub | Odometry topic (open-loop, dead-reckoned); no `odom -> base_link` TF is published by the controller |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | sub | Arm trajectory command |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | action | FollowJointTrajectory action server |
| `/arm_joint_temperatures` | `std_msgs/msg/Float64MultiArray` | pub | Arm servo temperatures in C (order = `arm_joints`) |
| `/joint_states` | `sensor_msgs/msg/JointState` | pub | Combined joint states (all joints) |

## Joints

| Joint | Type | Command | State |
|---|---|---|---|
| `wheel_1_joint` | wheel (left front) | velocity | position, velocity |
| `wheel_2_joint` | wheel (left rear) | velocity | position, velocity |
| `wheel_3_joint` | wheel (right front) | velocity | position, velocity |
| `wheel_4_joint` | wheel (right rear) | velocity | position, velocity |
| `arm_1_joint` | arm base | position | position, velocity |
| `arm_2_joint` | arm shoulder | position | position, velocity |
| `gripper_joint` | gripper | position | position, velocity |
| `gripper_joint_2` | gripper mirror | - | position, velocity (mirrors `gripper_joint`, negated) |

## Launch

```bash
ros2 launch kros_car bringup.launch.py
```

### Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `enable_wheel` | `true` | Enable base controller |
| `enable_arm` | `true` | Enable arm controller |
| `model` | `kros_car_description/.../kros_car.xacro` | Robot URDF xacro path |
| `wheel_port` | `/configs/hardware.yaml` or package share `config/hardware.yaml`, overridable by launch arg | Wheel serial port xacro argument |
| `arm_port` | `/configs/hardware.yaml` or package share `config/hardware.yaml`, overridable by launch arg | Arm serial port xacro argument |
| `...` | | Remaining fields below are also exposed as launch arguments |

Serial port settings, motor IDs, directions, and other hardware parameters are defined in the `configs/hardware.yaml` and exposed through `bringup.launch.py` as launch arguments.

Config precedence is:

1. explicit launch argument
2. `/configs/*.yaml`
3. packaged defaults in `share/kros_car/config`

Example:

Override hardware settings at launch time:

```bash
ros2 launch kros_car bringup.launch.py \
  wheel_port:=/dev/ttyUSB0 \
  arm_port:=/dev/ttyUSB1 \
  wheel_baudrate:=115200
```

Override controller settings inside a container without touching the installed package:

```bash
docker run -v ./config:/configs ...
```

### Hardware Parameters

These are set in the `configs/hardware.yaml`:

| Parameter | Default | Description |
|---|---|---|
| `wheel_port` | `/dev/usb_wheel` | Wheel serial port |
| `wheel_baudrate` | `115200` | Wheel serial baud rate |
| `wheel_motor_ids` | `1,2,3,4` | Wheel motor IDs (CSV) |
| `wheel_motor_directions` | `1,1,-1,-1` | Motor direction multipliers (CSV) |
| `max_rpm` | `100` | Max wheel RPM |
| `acceleration_act` | `1` | Wheel acceleration parameter |
| `arm_port` | `/dev/usb_robot_arm` | Arm serial port |
| `arm_baudrate` | `115200` | Arm serial baud rate |
| `arm_servo_ids` | `1,2,3` | Arm servo IDs (CSV) |
| `default_move_time_ms` | `80` | Default servo move duration (ms) |
| `arm_read_timeout_ms` | `80` | Servo read timeout (ms) |
| `arm_temp_publish_interval_cycles` | `10` | Publish/read temperature every N arm poll cycles |
| `gripper_max_ticks` | `1000` | Safety clamp for `gripper_joint` command (0-1000) |
| `gripper_stall_error_ticks` | `30` | Min (position - command) ticks to consider over-grip |
| `gripper_stall_trigger_ms` | `500` | How long over-grip must persist before protection engages |
| `gripper_stall_motion_epsilon_ticks` | `3` | Position-change threshold treated as "not moving" |
| `gripper_stall_backoff_ticks` | `10` | Auto-open offset when stall protection engages |
| `gripper_stall_release_delta_ticks` | `20` | Open-command margin required to release stall protection |
| `gripper_overheat_temp_c` | `70` | Gripper overheat threshold (C) |
| `gripper_overheat_hysteresis_c` | `5` | Overheat recovery hysteresis (C) |
| `wheel_joints` | `wheel_1_joint,...` | Wheel joint names (CSV) |
| `arm_joints` | `arm_1_joint,...` | Arm joint names (CSV) |

Arm move timing note:
- The hardware plugin computes effective command time as `max(default_move_time_ms, control_period_ms)`.
- With `update_rate: 15` (~66 ms period) and `default_move_time_ms: 80`, normal arm commands use 80 ms.

## Usage

Drive the base:

```bash
ros2 topic pub /base_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.2}, angular: {z: 0.4}}}"
```

Command the arm (action):

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['arm_1_joint','arm_2_joint','gripper_joint'], \
  points: [{positions: [1.0, 1.1, 2.0], time_from_start: {sec: 1}}]}}"
```

Command the arm (topic):

```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['arm_1_joint','arm_2_joint','gripper_joint'], \
  points: [{positions: [1.0, 1.1, 2.0], time_from_start: {sec: 1}}]}"
```

Introspect:

```bash
ros2 topic echo /joint_states
ros2 topic echo /arm_joint_temperatures
```

## Controllers

Defined in `config/controllers.yaml` (update rate: 15 Hz):

- **base_controller** (`diff_drive_controller`) — left: `wheel_1_joint`, `wheel_2_joint`; right: `wheel_3_joint`, `wheel_4_joint`. Limits: 0.8 m/s linear, 1.8 rad/s angular.
- **arm_controller** (`joint_trajectory_controller`) — `arm_1_joint`, `arm_2_joint`, `gripper_joint`. Position command, position + velocity state.
- **joint_state_broadcaster** — publishes combined `/joint_states`.

`base_controller` is configured with `enable_odom_tf: false`, so it publishes `/base_controller/odom` but does not own the `odom -> base_link` transform. That TF should be provided by the localization stack, such as a laser scan matcher.

## Threading Model

```
Controller Manager thread (15 Hz)
  +-- read()  — reads latest_arm_positions_ (mutex), dead-reckons wheels
  +-- write() — updates latest_wheel_rpms_ (mutex), updates latest arm command buffer (arm_cmd_mutex_)

Wheel TX thread (~12.5 Hz effective, 4 motors x 20ms gap)
  +-- reads latest_wheel_rpms_ (wheel_cmd_mutex_), writes to wheel serial

Arm TX thread
  +-- reads latest arm command buffer (arm_cmd_mutex_), writes servo commands (arm_serial_mutex_)

Arm Poll thread (~4 Hz per-servo, 3 servos x ~80-100ms each)
  +-- reads servo positions (arm_serial_mutex_), updates latest_arm_positions_ (arm_state_mutex_)
      reads temperature every ~10 cycles for overheat protection
```

## Notes

- Uses the Jazzy-native `ros2_control` API (framework-managed interfaces, `set_state`/`get_command`).
- Wheel odometry is open-loop (dead-reckoned). For closed-loop, add encoder feedback.
- The diff-drive controller does not publish `odom -> base_link` TF by default; external localization is expected to own that transform.
- Gripper overheat protection: force-opens at 70 C, re-enables at 65 C.
- Gripper anti-stall protection: if close command persists while gripper stops moving, it backs off slightly and blocks further closing until an open command is sent.
- Servo angle mapping: 0-1000 ticks = 0-240 deg. URDF joint limits should use 0 to ~4.189 rad.
