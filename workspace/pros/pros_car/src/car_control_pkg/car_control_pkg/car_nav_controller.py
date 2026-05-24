from car_control_pkg.nav2_utils import (
    cal_distance,
    calculate_diff_angle,
    get_yaw_from_quaternion,
)
from action_interface.action import NavGoal
import time
import math

class NavigationController:
    def __init__(self, car_control_node):
        self.car_control_node = car_control_node
        self.nav_end_flag = 0 

    def check_prerequisites(self):
        """Check if all prerequisites for navigation are met"""
        # Check if we have position data
        car_position, car_orientation = (
            self.car_control_node.get_car_position_and_orientation()
        )
        path_points = self.car_control_node.get_path_points()
        goal_pose = self.car_control_node.get_goal_pose()

        # Check data validity
        if not car_position or not path_points or not goal_pose:
            # Determine the specific error message based on what's missing
            message = (
                "Cannot obtain car position data"
                if not car_position
                else (
                    "No path points available for navigation"
                    if not path_points
                    else "No goal pose defined for navigation"
                )
            )

            return NavGoal.Result(success=False, message=message)
        else:
            # All prerequisites are met
            return car_position, car_orientation, path_points, goal_pose

    def data_init(self, car_position, car_orientation, goal_pose):
        return (
            [car_position.x, car_position.y],
            [car_orientation.z, car_orientation.w],
            [goal_pose.x, goal_pose.y],
        )

    def reset_index(self):
        self.index = 0

    # ------------------------------------------------------------------
    # Bridge navigation (Bridge_Nav mode)
    # ------------------------------------------------------------------
    def reset_bridge(self):
        """Initialise state for a fresh Bridge_Nav run."""
        self.index = 0
        self.bridge_phase = "APPROACH"
        self.bridge_seg_start = None
        self.bridge_seg_odom_start = None
        self.bridge_seg_last_progress = 0.0
        self.bridge_seg_last_progress_time = time.monotonic()
        self.bridge_foot_close_since = None
        self.bridge_max_approach_dist = 0.0
        self.bridge_params = self.car_control_node.get_bridge_params()
        self.car_control_node.get_logger().info(
            f"Bridge_Nav reset; params={self.bridge_params}"
        )

    @staticmethod
    def _heading_unit_vector(heading_deg):
        yaw = math.radians(heading_deg)
        return math.cos(yaw), math.sin(yaw)

    @staticmethod
    def _along_heading_progress(start_xy, current_xy, heading_deg):
        ux, uy = NavigationController._heading_unit_vector(heading_deg)
        dx = current_xy[0] - start_xy[0]
        dy = current_xy[1] - start_xy[1]
        return dx * ux + dy * uy

    def _start_cross_segment(self, node):
        self.bridge_seg_start = time.time()
        self.bridge_seg_last_progress = 0.0
        self.bridge_seg_last_progress_time = time.monotonic()
        pos, _ = node.get_odom_position_and_orientation()
        if pos is not None:
            self.bridge_seg_odom_start = (pos.x, pos.y)
        else:
            self.bridge_seg_odom_start = None

    def _cross_segment_progress(self, node, heading_deg):
        if self.bridge_seg_odom_start is None:
            return None
        pos, _ = node.get_odom_position_and_orientation()
        if pos is None:
            return None
        return self._along_heading_progress(
            self.bridge_seg_odom_start, (pos.x, pos.y), heading_deg
        )

    def _update_cross_progress(self, progress, min_progress, stuck_sec):
        if progress is None:
            return False
        if progress >= self.bridge_seg_last_progress + min_progress:
            self.bridge_seg_last_progress = progress
            self.bridge_seg_last_progress_time = time.monotonic()
        return (time.monotonic() - self.bridge_seg_last_progress_time) > stuck_sec

    def _advance_cross_segment(self, node, phase, next_phase, progress, target_dist):
        node.get_logger().info(
            f"Bridge_Nav: {phase} done (odom={progress:.2f}/{target_dist:.2f} m) -> {next_phase}"
        )
        self.bridge_phase = next_phase
        if next_phase != "DONE":
            self._start_cross_segment(node)

    @staticmethod
    def _normalize_deg(angle):
        """Normalise an angle in degrees to [-180, 180)."""
        return (angle + 180.0) % 360.0 - 180.0

    def _choose_drive_action(self, diff_angle, forward_thresh=20.0, median_thresh=30.0):
        """
        Turn toward a target bearing, then drive forward when aligned.
        Used by APPROACH (bearing to foot) and manual path follow.
        """
        if -forward_thresh < diff_angle < forward_thresh:
            return "FORWARD"
        if diff_angle > 0:
            return (
                "COUNTERCLOCKWISE_ROTATION_MEDIAN"
                if diff_angle < median_thresh
                else "COUNTERCLOCKWISE_ROTATION"
            )
        return (
            "CLOCKWISE_ROTATION_MEDIAN"
            if diff_angle > -median_thresh
            else "CLOCKWISE_ROTATION"
        )

    def bridge_nav(self):
        """
        One tick of the bridge-crossing state machine. Returns None while the
        mission is ongoing and a NavGoal.Result when finished.

        Phases: APPROACH (drive to foot) -> ALIGN (rotate to bridge heading)
        -> CROSS_UP / CROSS_PLATFORM / CROSS_DOWN (open-loop timed) -> DONE.
        """
        if not hasattr(self, "bridge_phase"):
            self.reset_bridge()

        p = self.bridge_params
        node = self.car_control_node

        if self.bridge_phase == "APPROACH":
            # Direct bearing navigation to the foot (wheels only — no /goal_pose so
            # Nav2 bt_navigator does not also publish conflicting /cmd_vel).
            car_position, car_orientation = node.get_car_position_and_orientation()
            if car_position is None or car_orientation is None:
                node.get_logger().warn(
                    "APPROACH: waiting for TF map→base_footprint...",
                    throttle_duration_sec=2.0,
                )
                return None

            dist = cal_distance(
                [car_position.x, car_position.y], [p["foot_x"], p["foot_y"]]
            )
            self.bridge_max_approach_dist = max(self.bridge_max_approach_dist, dist)

            if (
                dist < p["foot_thresh"]
                and self.bridge_max_approach_dist >= p["foot_min_approach"]
            ):
                if self.bridge_foot_close_since is None:
                    self.bridge_foot_close_since = time.monotonic()
                elif (
                    time.monotonic() - self.bridge_foot_close_since
                    >= p["foot_hold_sec"]
                ):
                    node.publish_control("STOP")
                    self.bridge_phase = "ALIGN"
                    self.bridge_foot_close_since = None
                    node.get_logger().info(
                        f"Bridge_Nav: reached foot (dist={dist:.2f} m, "
                        f"held {p['foot_hold_sec']:.1f}s); aligning"
                    )
                    return None
            else:
                self.bridge_foot_close_since = None

            diff_angle = calculate_diff_angle(
                [car_position.x, car_position.y],
                [car_orientation.z, car_orientation.w],
                [p["foot_x"], p["foot_y"]],
            )
            action = self._choose_drive_action(diff_angle)
            rotate = (
                "forward"
                if action == "FORWARD"
                else ("CCW" if "COUNTER" in action else "CW")
            )
            node.get_logger().info(
                f"APPROACH: dist={dist:.2f} m, bearing_err={diff_angle:.1f}° → {rotate}",
                throttle_duration_sec=2.0,
            )
            node.publish_control(action)
            return None

        if self.bridge_phase == "ALIGN":
            _, car_orientation = node.get_car_position_and_orientation()
            if car_orientation is None:
                return None
            current_yaw = get_yaw_from_quaternion(
                car_orientation.z, car_orientation.w
            )
            diff = self._normalize_deg(p["heading_deg"] - current_yaw)
            if abs(diff) < p["align_tol"]:
                node.publish_control("STOP")
                self.bridge_phase = "CROSS_UP"
                self._start_cross_segment(node)
                node.get_logger().info(
                    f"ALIGN: diff={diff:.1f}° → close enough; starting climb"
                )
                return None
            if diff > 0:
                action = (
                    "COUNTERCLOCKWISE_ROTATION_MEDIAN"
                    if abs(diff) < 30
                    else "COUNTERCLOCKWISE_ROTATION"
                )
                rotate_dir = "CCW"
            else:
                action = (
                    "CLOCKWISE_ROTATION_MEDIAN"
                    if abs(diff) < 30
                    else "CLOCKWISE_ROTATION"
                )
                rotate_dir = "CW"
            node.get_logger().info(
                f"ALIGN: diff={diff:.1f}° → rotating {rotate_dir}",
                throttle_duration_sec=2.0,
            )
            node.publish_control(action)
            return None

        if self.bridge_phase in ("CROSS_UP", "CROSS_PLATFORM", "CROSS_DOWN"):
            segments = {
                "CROSS_UP": (p["up_action"], p["up_dist"], p["up_sec"], "CROSS_PLATFORM"),
                "CROSS_PLATFORM": (
                    p["platform_action"],
                    p["platform_dist"],
                    p["platform_sec"],
                    "CROSS_DOWN",
                ),
                "CROSS_DOWN": (
                    p["down_action"],
                    p["down_dist"],
                    p["down_sec"],
                    "DONE",
                ),
            }
            action, target_dist, fallback_sec, next_phase = segments[self.bridge_phase]
            node.publish_control(action)

            elapsed = time.time() - self.bridge_seg_start
            progress = self._cross_segment_progress(node, p["heading_deg"])
            stuck = self._update_cross_progress(
                progress, p["cross_min_progress"], p["cross_stuck_sec"]
            )

            if progress is not None:
                node.get_logger().info(
                    f"{self.bridge_phase}: odom={progress:.2f}/{target_dist:.2f} m, "
                    f"t={elapsed:.1f}s",
                    throttle_duration_sec=2.0,
                )
                if progress >= target_dist:
                    self._advance_cross_segment(
                        node, self.bridge_phase, next_phase, progress, target_dist
                    )
                    return None
                if stuck:
                    node.get_logger().warn(
                        f"{self.bridge_phase}: no odom progress — still driving "
                        f"(SLAM may drift on ramp)",
                        throttle_duration_sec=2.0,
                    )
                if elapsed >= p["cross_max_sec"]:
                    node.get_logger().warn(
                        f"{self.bridge_phase}: max time {p['cross_max_sec']:.0f}s — "
                        f"advancing at odom={progress:.2f} m"
                    )
                    self._advance_cross_segment(
                        node, self.bridge_phase, next_phase, progress, target_dist
                    )
                return None

            # Fallback when odom TF is missing: timed segments only.
            node.get_logger().warn(
                f"{self.bridge_phase}: odom unavailable — using timed fallback "
                f"({fallback_sec:.0f}s)",
                throttle_duration_sec=3.0,
            )
            if elapsed >= fallback_sec:
                node.get_logger().info(
                    f"Bridge_Nav: {self.bridge_phase} done (timed) -> {next_phase}"
                )
                self.bridge_phase = next_phase
                if next_phase != "DONE":
                    self._start_cross_segment(node)
            return None

        # DONE
        for _ in range(5):
            node.publish_control("STOP")
            time.sleep(0.05)
        node.clear_plan()
        node.clear_goal_pose()
        node.get_logger().info("Bridge_Nav: bridge traversed")
        return NavGoal.Result(success=True, message="bridge traversed")

    def customize_nav(self):
        result = self.check_prerequisites()
        coordinate = self.car_control_node.get_latest_object_coordinates()
        if coordinate == {} or not coordinate:
            if self.nav_end_flag == 0:
                self.signal = self.manual_nav()
            else:
                if self.nav_end_flag == 1:
                    self.car_control_node.clear_plan()
                    self.car_control_node.clear_goal_pose()
                    self.car_control_node.publish_control("COUNTERCLOCKWISE_ROTATION_SLOW")
            # self.car_control_node.publish_control("STOP")                
        else:
            self.nav_end_flag = 0
            y_offset = coordinate["ball"][1]
            object_depth = coordinate["ball"][0]
            if object_depth < 0.3:
                for i in range(10):
                    self.car_control_node.publish_control("STOP")
                    time.sleep(0.1)
                self.car_control_node.clear_plan()
                self.car_control_node.clear_goal_pose()
                return NavGoal.Result(
                    success=True,
                    message="Navigation goal reached successfully. Final distance",
                )
            action = self.choose_action_y_offset(y_offset,object_depth)
            self.car_control_node.publish_control(action)
            
        # print(self.car_control_node.get_latest_object_coordinates())
    def choose_action_y_offset(self, y_offset, object_depth):
        if object_depth >= 0.5:
            limit = 0.5
        elif object_depth <= 0.5:
            limit = 0.1
        if y_offset > -limit and y_offset < limit:
            return "FORWARD_SLOW"
            self.car_control_node.publish_control("FORWARD_SLOW")
        elif y_offset >= limit: # 物體在左
            return "COUNTERCLOCKWISE_ROTATION_SLOW"
            self.car_control_node.publish_control("COUNTERCLOCKWISE_ROTATION_SLOW")
        elif y_offset <= -limit:
            return "CLOCKWISE_ROTATION_SLOW"
            self.car_control_node.publish_control("CLOCKWISE_ROTATION_SLOW")

    def manual_nav(self):
        result = self.check_prerequisites()

        if isinstance(result, NavGoal.Result):
            # 有錯誤就直接回傳結果，不繼續導航流程
            return result

        # 正常情況才解包
        car_position, car_orientation, path_points, goal_pose = result
        car_position, car_orientation, goal_pose = self.data_init(
            car_position, car_orientation, goal_pose
        )

        target_distance = cal_distance(car_position, goal_pose)
        if target_distance < 0.5:
            self.nav_end_flag = 1
            self.car_control_node.publish_control("STOP")
            return NavGoal.Result(
                success=True,
                message="Navigation goal reached successfully. Final distance",
            )
        else:
            target_points, orientation_points = self.get_next_target_point(
                car_position=car_position, path_points=path_points
            )
            diff_angle = calculate_diff_angle(
                car_position, car_orientation, target_points
            )
            action_key = self._choose_drive_action(diff_angle)
            self.car_control_node.publish_control(action_key)

    def choose_action(self, diff_angle):
        return self._choose_drive_action(diff_angle)

    def get_next_target_point(
        self, car_position, path_points, min_required_distance=0.5
    ):
        """
        Get the next target point along the path that is at least min_required_distance away
        from the car_position. Returns a tuple of ([target_x, target_y], [orientation_x, orientation_y])
        or (None, None) if no valid target is found.
        """
        logger = self.car_control_node.get_logger()

        if not path_points:
            logger.error("Error: No path points available!")
            return None, None

        # Ensure self.index is initialized
        if not hasattr(self, "index"):
            self.index = 0

        # Iterate over the remaining path points starting from the current index
        for idx in range(self.index, len(path_points)):
            point = path_points[idx]
            try:
                pos = point["position"]
                orient = point["orientation"]
                target_x, target_y = pos[0], pos[1]
                orientation_x, orientation_y = orient[0], orient[1]
            except (KeyError, IndexError, TypeError) as e:
                logger.error(f"Invalid path point format at index {idx}: {e}")
                continue

            distance_to_target = cal_distance(car_position, (target_x, target_y))
            if distance_to_target >= min_required_distance:
                # Update self.index to current valid point index for future calls
                self.index = idx
                logger.debug(
                    f"Found valid target point at index {idx} with distance {distance_to_target:.2f}"
                )
                return [target_x, target_y], [orientation_x, orientation_y]
            else:
                logger.debug(
                    f"Skipping point at index {idx}: distance {distance_to_target:.2f} is less than required {min_required_distance}"
                )

        # If no intermediate point meets the criteria, return the final point regardless of distance.
        try:
            last_point = path_points[-1]
            pos = last_point["position"]
            orient = last_point["orientation"]
            last_x, last_y = pos[0], pos[1]
            last_ox, last_oy = orient[0], orient[1]
            logger.info(
                "No point met the minimum distance requirement; using the last point as target."
            )
            self.index = len(path_points) - 1
            return [last_x, last_y], [last_ox, last_oy]
        except (KeyError, IndexError, TypeError) as e:
            logger.error(f"Invalid format for last path point: {e}")

        logger.warning("No valid target point found.")
        return None, None
