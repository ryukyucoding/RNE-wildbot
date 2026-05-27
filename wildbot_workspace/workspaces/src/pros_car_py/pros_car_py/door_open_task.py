"""
Real-car door opening task.

Usage inside the wildbot container:
  cd /workspaces
  colcon build --packages-select pros_car_py --symlink-install
  source install/setup.bash
  ros2 run pros_car_py door_open

Expected supporting services:
  - Wildbot hardware stack publishes /base_controller/odom, /scan, camera topics.
  - YOLO publishes Float32MultiArray /yolo/target_info = [found, depth_m, pixel_offset].
  - YOLO listens to /target_label, so this node can request the "knob" class.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from pros_car_py.arm_controller_2D import ArmController
from pros_car_py.car_controller import CarController
from pros_car_py.data_processor import DataProcessor
from pros_car_py.nav_processing import Nav2Processing
from pros_car_py.ros_communicator import RosCommunicator


DEFAULT_LENGTH_M = 0.85
DEFAULT_WIDTH_M = 2.95
DEFAULT_TURN_DEG = 48.5
TURN_TOLERANCE_DEG = 2.0
FORWARD_ACTION = "FORWARD_SLOW"
TURN_RIGHT_ACTION = "CLOCKWISE_ROTATION"
TURN_LEFT_ACTION = "COUNTERCLOCKWISE_ROTATION"
SEGMENT_TIMEOUT_SEC = 30.0
ODOM_WAIT_TIMEOUT_SEC = 10.0

SEARCH_MAX_ITER = 600
VS_BASE_SPEED = 60.0
VS_KP_STEER = 1.5
VS_MAX_STEER = 250.0
VS_MIN_STEER = 120.0
VS_TARGET_PIXEL_OFFSET = 0.0
VS_PATIENCE_SECS = 3.0
VS_ALIGN_PIXEL_TOL = 20.0
VS_ALIGN_KP = 0.6
VS_ALIGN_TIMEOUT = 5.0
DEPTH_EMA_ALPHA = 0.3

ARM_MAX_REACH = 0.17
PRESS_ABOVE_OFFSET = 0.05
CAMERA_X_OFFSET = -0.15
KNOB_Z_HEIGHT = 0.05

DOOR_OPEN_ACTION = "FORWARD_SLOW"


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
    cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(theta: float) -> float:
    return math.atan2(math.sin(theta), math.cos(theta))


class DoorOpenState:
    ARM_RAISE_INIT = 0
    PRE_APPROACH = 1
    SEARCH_HANDLE = 2
    VISUAL_SERVO_APPROACH = 3
    ARM_AIM = 4
    PRESS_DOWN = 5
    OPEN_DOOR = 6
    DONE = 7
    ERROR = 99


class DoorOpenTask:
    def __init__(
        self,
        car_controller: CarController,
        arm_controller: ArmController,
        nav_processing: Nav2Processing,
        data_processor: DataProcessor,
        ros_communicator: RosCommunicator,
    ):
        self.car = car_controller
        self.arm = arm_controller
        self.nav = nav_processing
        self.dp = data_processor
        self.rc = ros_communicator

        self._declare_parameters()
        self._load_parameters()

        self.state = DoorOpenState.ARM_RAISE_INIT
        self._iter = 0
        self._press_count = 0
        self._open_start: float | None = None
        self._last_knob_depth: float | None = None
        self._depth_ema: float | None = None
        self._vs_lost_since: float | None = None
        self._fine_align_since: float | None = None
        self._last_vs_velocities = [0.0, 0.0, 0.0, 0.0]
        self._last_knob_x = 0.0
        self._last_knob_z = 0.0
        self._abort = False

    def _declare_parameters(self) -> None:
        self.rc.declare_parameter("length_m", DEFAULT_LENGTH_M)
        self.rc.declare_parameter("width_m", DEFAULT_WIDTH_M)
        self.rc.declare_parameter("turn_deg", DEFAULT_TURN_DEG)
        self.rc.declare_parameter("forward_factor_a", 2.0)
        self.rc.declare_parameter("forward_factor_b", 0.5)
        self.rc.declare_parameter("run_pre_approach", True)
        self.rc.declare_parameter("yolo_target_label", "knob")
        self.rc.declare_parameter("vs_stop_distance", 0.30)
        self.rc.declare_parameter("press_elbow_drop_deg", 30.0)
        self.rc.declare_parameter("open_door_duration", 2.5)
        self.rc.declare_parameter("gripper_open_deg", 240.0)
        self.rc.declare_parameter("gripper_close_deg", 168.0)
        self.rc.declare_parameter("gripper_retreat_deg", 170.0)

    def _load_parameters(self) -> None:
        self.length_m = max(0.1, self._double_param("length_m"))
        self.width_m = max(0.1, self._double_param("width_m"))
        self.turn_deg = min(max(30.0, self._double_param("turn_deg")), 90.0)
        self.forward_factor_a = max(0.0, self._double_param("forward_factor_a"))
        self.forward_factor_b = max(0.0, self._double_param("forward_factor_b"))
        self.run_pre_approach = self._bool_param("run_pre_approach")
        self.yolo_target_label = self._string_param("yolo_target_label")
        self.vs_stop_distance = max(0.1, self._double_param("vs_stop_distance"))
        self.press_elbow_drop_deg = max(0.0, self._double_param("press_elbow_drop_deg"))
        self.open_door_duration = max(0.1, self._double_param("open_door_duration"))
        self.gripper_open_deg = self._safe_gripper_deg(
            self._double_param("gripper_open_deg")
        )
        self.gripper_close_deg = self._safe_gripper_deg(
            self._double_param("gripper_close_deg")
        )
        self.gripper_retreat_deg = self._safe_gripper_deg(
            self._double_param("gripper_retreat_deg")
        )

    def _double_param(self, name: str) -> float:
        return float(self.rc.get_parameter(name).value)

    def _bool_param(self, name: str) -> bool:
        value = self.rc.get_parameter(name).value
        if isinstance(value, str):
            return value.lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _string_param(self, name: str) -> str:
        return str(self.rc.get_parameter(name).value)

    def _safe_gripper_deg(self, requested_deg: float) -> float:
        max_angle = float(self.arm.joint_limits[2]["max_angle"])
        safe_min = 168.0
        return max(safe_min, min(float(requested_deg), max_angle))

    def step(self) -> bool:
        if self.state == DoorOpenState.ARM_RAISE_INIT:
            return self._state_arm_raise_init()
        if self.state == DoorOpenState.PRE_APPROACH:
            return self._state_pre_approach()
        if self.state == DoorOpenState.SEARCH_HANDLE:
            return self._state_search()
        if self.state == DoorOpenState.VISUAL_SERVO_APPROACH:
            return self._state_visual_servo_approach()
        if self.state == DoorOpenState.ARM_AIM:
            return self._state_arm_aim()
        if self.state == DoorOpenState.PRESS_DOWN:
            return self._state_press_down()
        if self.state == DoorOpenState.OPEN_DOOR:
            return self._state_open_door()
        if self.state == DoorOpenState.DONE:
            return True
        if self.state == DoorOpenState.ERROR:
            print("[DoorOpenTask] Task failed; stopping base.")
            self.car.update_action("STOP")
            return True
        return False

    def run_blocking(self, spin_interval: float = 0.05) -> None:
        print("[DoorOpenTask] Starting real-car door open task.")
        print(
            "[DoorOpenTask] Pre-approach config: "
            f"{self.forward_factor_a:.2f} * length({self.length_m:.2f}m), "
            f"right {self.turn_deg:.1f}deg, "
            f"{self.forward_factor_b:.2f} * width({self.width_m:.2f}m), "
            f"left {self.turn_deg:.1f}deg"
        )
        while rclpy.ok() and not self.step():
            time.sleep(spin_interval)
        print(f"[DoorOpenTask] Finished with state={self.state}.")

    def stop(self) -> None:
        self._abort = True
        self.car.update_action("STOP")
        self.rc.publish_target_label("")

    def _state_arm_raise_init(self) -> bool:
        print("[State 0] Raising arm to a safe initial pose.")
        try:
            self.arm.joint_angles = [
                self.arm.joint_limits[0]["init"],
                self.arm.joint_limits[1]["init"],
                self.gripper_open_deg,
            ]
            for _ in range(5):
                self.arm._clamp_and_publish()
                self.arm._visualize_arm_lines()
                time.sleep(0.2)
        except Exception as exc:
            print(f"[State 0] Arm initialization warning: {exc}")

        next_state = (
            DoorOpenState.PRE_APPROACH
            if self.run_pre_approach
            else DoorOpenState.SEARCH_HANDLE
        )
        self._transition(next_state)
        return False

    def _state_pre_approach(self) -> bool:
        if self._iter > 0:
            return False
        self._iter = 1

        if not self._wait_for_odom(ODOM_WAIT_TIMEOUT_SEC):
            print(f"[PreApproach] No odom after {ODOM_WAIT_TIMEOUT_SEC:.1f}s.")
            self._transition(DoorOpenState.ERROR)
            return False

        first_forward = self.length_m * self.forward_factor_a
        second_forward = self.width_m * self.forward_factor_b
        print(
            "[PreApproach] Running scripted approach: "
            f"forward {first_forward:.2f}m, right {self.turn_deg:.1f}deg, "
            f"forward {second_forward:.2f}m, left {self.turn_deg:.1f}deg."
        )

        ok = (
            self._drive_forward(first_forward, 1)
            and self._turn(self.turn_deg, 1, "right")
            and self._drive_forward(second_forward, 2)
            and self._turn(self.turn_deg, 2, "left")
        )
        self.car.update_action("STOP")

        if ok:
            print("[PreApproach] Completed; starting knob search.")
            self._transition(DoorOpenState.SEARCH_HANDLE)
        else:
            print("[PreApproach] Failed; stopping task.")
            self._transition(DoorOpenState.ERROR)
        return False

    def _wait_for_odom(self, timeout_sec: float) -> bool:
        t0 = time.monotonic()
        while rclpy.ok() and not self._abort and time.monotonic() - t0 < timeout_sec:
            if self.rc.get_latest_odom() is not None:
                return True
            time.sleep(0.05)
        return False

    def _get_xy_yaw(self) -> tuple[float, float, float] | None:
        odom = self.rc.get_latest_odom()
        if odom is None:
            return None
        pose = odom.pose.pose
        return (
            float(pose.position.x),
            float(pose.position.y),
            _yaw_from_quat(pose.orientation),
        )

    @staticmethod
    def _segment_timeout_for(distance_m: float) -> float:
        return max(SEGMENT_TIMEOUT_SEC, distance_m / 0.12 * 1.5)

    def _drive_forward(self, distance_m: float, segment_idx: int) -> bool:
        segment_timeout_sec = self._segment_timeout_for(distance_m)
        start = self._get_xy_yaw()
        if start is None:
            print(f"[PreApproach][forward {segment_idx}] No odom; aborting.")
            return False

        x0, y0, _ = start
        t0 = time.monotonic()
        last_log = 0.0

        while rclpy.ok() and not self._abort:
            if time.monotonic() - t0 > segment_timeout_sec:
                self.car.update_action("STOP")
                print(f"[PreApproach][forward {segment_idx}] Timeout.")
                return False

            cur = self._get_xy_yaw()
            if cur is None:
                time.sleep(0.05)
                continue

            x, y, _ = cur
            traveled = math.hypot(x - x0, y - y0)
            now = time.monotonic()
            if now - last_log >= 1.0:
                print(
                    f"[PreApproach][forward {segment_idx}] "
                    f"{traveled:.3f}m / {distance_m:.3f}m"
                )
                last_log = now

            if traveled >= distance_m:
                self.car.update_action("STOP")
                print(
                    f"[PreApproach][forward {segment_idx}] "
                    f"done, actual={traveled:.3f}m"
                )
                return True

            self.car.update_action(FORWARD_ACTION)
            time.sleep(0.05)

        self.car.update_action("STOP")
        return False

    def _turn(self, turn_deg: float, turn_idx: int, direction: str) -> bool:
        sign = -1.0 if direction == "right" else 1.0
        action = TURN_RIGHT_ACTION if direction == "right" else TURN_LEFT_ACTION
        target_rad = sign * math.radians(turn_deg)
        tol_rad = math.radians(TURN_TOLERANCE_DEG)
        start = self._get_xy_yaw()
        if start is None:
            print(f"[PreApproach][turn {turn_idx}] No odom; aborting.")
            return False

        _, _, start_yaw = start
        accumulated = 0.0
        last_yaw = start_yaw
        t0 = time.monotonic()
        last_log = 0.0

        while rclpy.ok() and not self._abort:
            if time.monotonic() - t0 > SEGMENT_TIMEOUT_SEC:
                self.car.update_action("STOP")
                print(f"[PreApproach][turn {turn_idx}] Timeout.")
                return False

            cur = self._get_xy_yaw()
            if cur is None:
                time.sleep(0.05)
                continue

            _, _, yaw = cur
            dyaw = _normalize_angle(yaw - last_yaw)
            accumulated += dyaw
            last_yaw = yaw

            now = time.monotonic()
            if now - last_log >= 1.0:
                print(
                    f"[PreApproach][turn {turn_idx} {direction}] "
                    f"{math.degrees(accumulated):+.1f}deg / "
                    f"{math.degrees(target_rad):+.1f}deg"
                )
                last_log = now

            reached = (
                accumulated <= target_rad + tol_rad
                if direction == "right"
                else accumulated >= target_rad - tol_rad
            )
            if reached:
                self.car.update_action("STOP")
                print(
                    f"[PreApproach][turn {turn_idx} {direction}] "
                    f"done, actual={math.degrees(accumulated):+.1f}deg"
                )
                return True

            self.car.update_action(action)
            time.sleep(0.05)

        self.car.update_action("STOP")
        return False

    def _state_search(self) -> bool:
        if self._iter == 0:
            print(
                f"[State 1] Searching for door knob; target='{self.yolo_target_label}'."
            )
            self.rc.publish_target_label(self.yolo_target_label)
            time.sleep(0.3)

        yolo = self.dp.get_yolo_target_info()
        if yolo is not None and len(yolo) >= 3 and yolo[0] == 1:
            print("[State 1] Knob detected.")
            self.car.update_action("STOP")
            self._transition(DoorOpenState.VISUAL_SERVO_APPROACH)
            return False

        self.car.update_action("CLOCKWISE_ROTATION_SLOW")
        self._iter += 1

        if self._iter > SEARCH_MAX_ITER:
            print("[State 1] Search timeout.")
            self._transition(DoorOpenState.ERROR)

        time.sleep(0.05)
        return False

    def _get_front_lidar_min(self) -> float | None:
        lidar_msg = self.rc.latest_lidar
        if lidar_msg is None or not lidar_msg.ranges:
            return None

        front_half_angle = math.radians(15.0)
        valid: list[float] = []
        for idx, range_m in enumerate(lidar_msg.ranges):
            if not math.isfinite(range_m) or not (0.1 < range_m < 4.0):
                continue
            angle = _normalize_angle(
                lidar_msg.angle_min + idx * lidar_msg.angle_increment
            )
            if abs(angle) <= front_half_angle:
                valid.append(float(range_m))
        return min(valid) if valid else None

    def _state_visual_servo_approach(self) -> bool:
        if self._iter == 0:
            print(
                "[State 2] Starting visual servo approach; "
                f"stop_distance={self.vs_stop_distance:.2f}m."
            )
            self._last_vs_velocities = [0.0, 0.0, 0.0, 0.0]
            self._vs_lost_since = None
            self._fine_align_since = None

        yolo = self.dp.get_yolo_target_info()
        lidar_dist = self._get_front_lidar_min()

        if yolo is None or len(yolo) < 3 or yolo[0] == 0:
            if self._vs_lost_since is None:
                self._vs_lost_since = time.time()
                print(
                    "[State 2] YOLO temporarily lost; "
                    f"holding velocity for {VS_PATIENCE_SECS:.1f}s."
                )
            lost_dur = time.time() - self._vs_lost_since
            if lost_dur < VS_PATIENCE_SECS:
                self.rc.publish_raw_car_control(self._last_vs_velocities)
                self._iter += 1
                return False
            print("[State 2] YOLO lost too long; returning to search.")
            self._vs_lost_since = None
            self.car.update_action("STOP")
            self._transition(DoorOpenState.SEARCH_HANDLE)
            return False

        self._vs_lost_since = None
        pixel_offset = float(yolo[2])
        error = pixel_offset - VS_TARGET_PIXEL_OFFSET

        distance_reached = lidar_dist is not None and lidar_dist <= self.vs_stop_distance
        if yolo[1] > 0:
            depth = float(yolo[1])
            self._depth_ema = (
                depth
                if self._depth_ema is None
                else DEPTH_EMA_ALPHA * depth + (1.0 - DEPTH_EMA_ALPHA) * self._depth_ema
            )
            if self._depth_ema <= self.vs_stop_distance:
                distance_reached = True

        if distance_reached:
            if self._fine_align_since is None:
                self._fine_align_since = time.time()
                self._last_knob_depth = lidar_dist if lidar_dist is not None else self._depth_ema
                print(
                    "[State 2] Stop distance reached; fine aligning "
                    f"(error={error:.1f}px)."
                )

            align_elapsed = time.time() - self._fine_align_since
            if abs(error) <= VS_ALIGN_PIXEL_TOL:
                print(f"[State 2] Fine alignment done, error={error:.1f}px.")
                self.car.update_action("STOP")
                self._transition(DoorOpenState.ARM_AIM)
                return False
            if align_elapsed > VS_ALIGN_TIMEOUT:
                print(
                    "[State 2] Fine alignment timeout; "
                    f"continuing with error={error:.1f}px."
                )
                self.car.update_action("STOP")
                self._transition(DoorOpenState.ARM_AIM)
                return False

            rotate_output = VS_ALIGN_KP * error
            if abs(rotate_output) < VS_MIN_STEER:
                rotate_output = VS_MIN_STEER if rotate_output > 0 else -VS_MIN_STEER
            rotate_output = max(-VS_MAX_STEER, min(VS_MAX_STEER, rotate_output))

            v_left = rotate_output
            v_right = -rotate_output
            velocities = [v_left, v_right, v_left, v_right]
            self.rc.publish_raw_car_control(velocities)
            self._last_vs_velocities = velocities

            if self._iter % 10 == 0:
                print(
                    f"[Align] error={error:.1f}px, L={v_left:.0f}, "
                    f"R={v_right:.0f}, elapsed={align_elapsed:.1f}s"
                )
            self._iter += 1
            return False

        steer_output = VS_KP_STEER * error
        if abs(error) > 5.0 and abs(steer_output) < VS_MIN_STEER:
            steer_output = VS_MIN_STEER if steer_output > 0 else -VS_MIN_STEER
        steer_output = max(-VS_MAX_STEER, min(VS_MAX_STEER, steer_output))

        v_left = VS_BASE_SPEED + steer_output
        v_right = VS_BASE_SPEED - steer_output
        velocities = [v_left, v_right, v_left, v_right]
        self.rc.publish_raw_car_control(velocities)
        self._last_vs_velocities = velocities

        if self._iter % 20 == 0:
            lidar_text = f"{lidar_dist:.2f}m" if lidar_dist is not None else "none"
            print(
                f"[VS] offset={pixel_offset:.1f}px, steer={steer_output:.1f}, "
                f"L={v_left:.0f}, R={v_right:.0f}, lidar={lidar_text}"
            )

        self._iter += 1
        return False

    def _state_arm_aim(self) -> bool:
        if self._iter == 0:
            print("[State 3] Opening gripper and aiming 2D arm above knob.")
            self._set_gripper(self.gripper_open_deg)
            self._iter = 1
            time.sleep(0.3)

        check_depth = (
            self._last_knob_depth
            if self._last_knob_depth is not None
            else self.vs_stop_distance
        )
        arm_x = max(0.02, min(check_depth + CAMERA_X_OFFSET, ARM_MAX_REACH))
        x_target = arm_x
        z_target = KNOB_Z_HEIGHT
        x_above = x_target
        z_above = z_target + PRESS_ABOVE_OFFSET

        print(
            f"[State 3] depth={check_depth:.3f}m, "
            f"arm_target=({x_above:.3f}, {z_above:.3f})."
        )

        try:
            # Step 1: fold to the safe init posture (shoulder=-180°, elbow=0°).
            # Gripper tip lands at ~(0.095, -0.026) — forward and slightly below base.
            self.arm._smooth_move_to(
                [
                    self.arm.joint_limits[0]["init"],
                    self.arm.joint_limits[1]["init"],
                    None,
                ],
                step=5.0,
                delay=0.08,
            )
            # Step 2: raise shoulder to its max (0°) while elbow stays folded.
            # th1 = radians(0° - 270°) = -270° ≡ 90°, so the shoulder link points
            # straight up and the gripper tip swings back to ~(-0.095, 0.026) —
            # entirely behind the robot, away from the door.
            # Verified: minimum link-segment distance to the knob is ≥ 6 cm throughout.
            self.arm._smooth_move_to(
                [
                    self.arm.joint_limits[0]["max_angle"],
                    self.arm.joint_limits[1]["init"],
                    None,
                ],
                step=5.0,
                delay=0.08,
            )
            # Step 3: extend from the raised posture down to the above-knob target.
            # The arm descends from high above (z>0.10 throughout) so the link can
            # never approach the knob from below.
            # Verified: minimum link-segment distance to the knob is ≥ 4.8 cm.
            self._move_arm_to_2d(x_above, z_above)
            self._last_knob_x = x_target
            self._last_knob_z = z_target
            self._transition(DoorOpenState.PRESS_DOWN)
        except Exception as exc:
            print(f"[State 3] Arm aim failed: {exc}")
            self._transition(DoorOpenState.ERROR)

        return False

    def _state_press_down(self) -> bool:
        if self._press_count != 0:
            return False

        print("[State 4] Closing gripper and pressing handle down.")
        try:
            self._set_gripper(self.gripper_close_deg)
            time.sleep(0.1)
            self._set_gripper(self.gripper_retreat_deg)
            time.sleep(0.3)

            elbow_idx = 1
            elbow_min = self.arm.joint_limits[elbow_idx]["min_angle"]
            elbow_target = max(
                elbow_min,
                self.arm.joint_angles[elbow_idx] - self.press_elbow_drop_deg,
            )
            print(
                f"[State 4] Dropping elbow by {self.press_elbow_drop_deg:.1f}deg "
                f"to {elbow_target:.1f}deg."
            )
            self.arm._smooth_move_to([None, elbow_target, None], step=3.0, delay=0.08)
            time.sleep(0.5)
            self._transition(DoorOpenState.OPEN_DOOR)
        except Exception as exc:
            print(f"[State 4] Press down failed: {exc}")
            self._transition(DoorOpenState.ERROR)

        self._press_count = 1
        return False

    def _state_open_door(self) -> bool:
        if self._open_start is None:
            print("[State 5] Pushing door open with the base.")
            self._open_start = time.time()

        elapsed = time.time() - self._open_start
        if elapsed < self.open_door_duration:
            self.car.update_action(DOOR_OPEN_ACTION)
        else:
            self.car.update_action("STOP")
            print("[State 5] Door open action complete.")
            self._transition(DoorOpenState.DONE)

        time.sleep(0.05)
        return False

    def _set_gripper(self, target_deg: float) -> None:
        self.arm.set_last_joint_angle(self._safe_gripper_deg(target_deg))

    def _move_arm_to_2d(self, x: float, z: float) -> None:
        if hasattr(self.arm, "move_to_2d_position"):
            self.arm.move_to_2d_position(x, z, step=4.0, delay=0.08)
            return
        deg1, deg2 = self.arm._calculate_2d_ik(x, z)
        self.arm._smooth_move_to([deg1, deg2, None], step=4.0, delay=0.08)

    def _transition(self, new_state: int) -> None:
        print(f"[FSM] {self.state} -> {new_state}")
        if new_state in (DoorOpenState.DONE, DoorOpenState.ERROR):
            self.rc.publish_target_label("")
            try:
                print("[FSM] Releasing gripper and resetting arm.")
                self._set_gripper(self.gripper_open_deg)
                time.sleep(0.5)
                self.arm.reset_arm()
            except Exception as exc:
                print(f"[FSM] Arm reset failed: {exc}")

        self.state = new_state
        self._iter = 0
        self._press_count = 0
        self._open_start = None


def main(args=None) -> None:
    rclpy.init(args=args)
    ros_communicator = RosCommunicator("door_open")
    executor = MultiThreadedExecutor()
    executor.add_node(ros_communicator)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    data_processor = DataProcessor(ros_communicator)
    nav_processing = Nav2Processing(ros_communicator, data_processor)
    car_controller = CarController(ros_communicator, nav_processing)
    arm_controller = ArmController(ros_communicator, data_processor)

    task = DoorOpenTask(
        car_controller=car_controller,
        arm_controller=arm_controller,
        nav_processing=nav_processing,
        data_processor=data_processor,
        ros_communicator=ros_communicator,
    )

    try:
        task.run_blocking(spin_interval=0.05)
    except KeyboardInterrupt:
        print("[door_open] Interrupted; stopping.")
        task.stop()
    finally:
        task.stop()
        executor.shutdown()
        ros_communicator.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
