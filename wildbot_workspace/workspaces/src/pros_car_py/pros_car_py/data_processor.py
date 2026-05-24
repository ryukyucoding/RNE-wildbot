# from geometry_msgs.msg impor
import math
import time

# LiDAR global constants (legacy downsample indices — may not match short /scan)
LIDAR_RANGE = 90
LIDAR_PER_SECTOR = 20
FRONT_LIDAR_INDICES = list(range(0, 16)) + list(range(-15, 0))  # front lidar indices
LEFT_LIDAR_INDICES = list(range(16, 46))  # left lidar indices
RIGHT_LIDAR_INDICES = list(range(-45, -15))  # right lidar indices

# Angle bins for obstacle guard (degrees, 0 = lidar forward)
LIDAR_FRONT_HALF_ANGLE_DEG = 35.0
LIDAR_SIDE_MIN_ANGLE_DEG = 38.0
LIDAR_SIDE_MAX_ANGLE_DEG = 72.0
# Scan hits with x < -REAR_AXLE_BEHIND_LIDAR_M (lidar frame) are rear-only, not forward.
REAR_AXLE_BEHIND_LIDAR_M = 0.22
LIDAR_REAR_MIN_ANGLE_DEG = 100.0
LIDAR_MIN_RANGE_M = 0.12
LIDAR_MAX_RANGE_M = 50.0
# Close returns must span at least this many degrees to count as a real obstacle.
LIDAR_CLOSE_OBSTACLE_M = 0.49
LIDAR_CLOSE_MIN_SPAN_DEG = 4.0
LIDAR_CLOSE_GAP_TOL_DEG = 2.5


class DataProcessor:
    def __init__(self, ros_communicator):
        self.ros_communicator = ros_communicator

    def get_processed_amcl_pose(self):
        amcl_pose_msg = self.ros_communicator.get_latest_amcl_pose()
        position = amcl_pose_msg.pose.pose.position
        orientation = amcl_pose_msg.pose.pose.orientation
        pose = [position.x, position.y, position.z]
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        return pose, quaternion

    def get_yolo_target_info(self):
        if self.ros_communicator.get_latest_yolo_target_info() is not None:
            return list(self.ros_communicator.get_latest_yolo_target_info().data)
        else:
            return None

    def get_camera_x_multi_depth(self):
        if self.ros_communicator.get_latest_camera_x_multi_depth() is not None:
            return list(self.ros_communicator.get_latest_camera_x_multi_depth().data)
        else:
            return None

    def get_obstacle_sector_depth(self):
        msg = self.ros_communicator.get_latest_obstacle_sector_depth()
        if msg is not None:
            return list(msg.data)
        return None

    @staticmethod
    def _pick_downsampled_ranges(ranges_180, indices):
        """Legacy index pick; skips out-of-range indices (Unity has fewer bins)."""
        n = len(ranges_180)
        if n == 0:
            return []
        out = []
        for i in indices:
            idx = i + n if i < 0 else i
            if 0 <= idx < n:
                out.append(ranges_180[idx])
        return out

    @staticmethod
    def _normalize_scan_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _robust_sector_distance(vals: list[float], percentile: float = 0.20) -> float | None:
        """
        Robust sector distance:
        - drop invalid / too-near noise / far outliers
        - reject spikes far below median (common with floor hits)
        - use low percentile (not absolute min)
        """
        ok = sorted(
            v
            for v in vals
            if math.isfinite(v)
            and LIDAR_MIN_RANGE_M < v < LIDAR_MAX_RANGE_M
        )
        if not ok:
            return None
        if len(ok) < 5:
            return ok[0]

        median = ok[len(ok) // 2]
        floor_cut = max(LIDAR_MIN_RANGE_M + 0.03, median * 0.62, median - 0.28)
        trimmed = [v for v in ok if v >= floor_cut]
        if len(trimmed) < 3:
            trimmed = ok

        idx = min(len(trimmed) - 1, max(0, int(len(trimmed) * percentile)))
        return trimmed[idx]

    @staticmethod
    def _lidar_best_close_run(
        close_hits: list[tuple[float, float]],
        gap_tol_deg: float = LIDAR_CLOSE_GAP_TOL_DEG,
    ) -> tuple[float, list[tuple[float, float]]]:
        """Longest contiguous run of close hits; span is end_angle - start_angle (deg)."""
        if not close_hits:
            return 0.0, []
        sorted_hits = sorted(close_hits, key=lambda t: t[0])
        best_span = 0.0
        best_run: list[tuple[float, float]] = []
        run = [sorted_hits[0]]
        for i in range(1, len(sorted_hits)):
            gap = math.degrees(sorted_hits[i][0] - sorted_hits[i - 1][0])
            if gap <= gap_tol_deg:
                run.append(sorted_hits[i])
            else:
                span = math.degrees(run[-1][0] - run[0][0])
                if span > best_span:
                    best_span = span
                    best_run = list(run)
                run = [sorted_hits[i]]
        span = math.degrees(run[-1][0] - run[0][0])
        if span > best_span:
            best_span = span
            best_run = list(run)
        return best_span, best_run

    def _lidar_sector_range_from_hits(
        self,
        hits: list[tuple[float, float]],
        close_m: float = LIDAR_CLOSE_OBSTACLE_M,
        min_span_deg: float = LIDAR_CLOSE_MIN_SPAN_DEG,
    ) -> tuple[float | None, float]:
        """
        Sector range with contiguous-close filter.

        Returns (range_m, close_span_deg). Close returns (< close_m) only count
        when they span >= min_span_deg; otherwise they are ignored as glancing noise.
        """
        if not hits:
            return None, 0.0

        close_hits = [
            (a, r)
            for a, r in hits
            if LIDAR_MIN_RANGE_M < r < close_m
        ]
        far_ranges = [
            r
            for _, r in hits
            if close_m <= r < LIDAR_MAX_RANGE_M
        ]

        if close_hits:
            span, run = self._lidar_best_close_run(close_hits)
            if span >= min_span_deg and run:
                vals = [r for _, r in run]
                robust = self._robust_sector_distance(vals)
                return robust, span
            if far_ranges:
                return self._robust_sector_distance(far_ranges), span
            return None, span

        all_ranges = [r for _, r in hits if LIDAR_MIN_RANGE_M < r < LIDAR_MAX_RANGE_M]
        if not all_ranges:
            return None, 0.0
        return self._robust_sector_distance(all_ranges), 0.0

    def _collect_lidar_forward_hits(self, lidar_msg):
        angle_min = lidar_msg.angle_min
        angle_increment = lidar_msg.angle_increment
        front_half = math.radians(LIDAR_FRONT_HALF_ANGLE_DEG)
        side_min = math.radians(LIDAR_SIDE_MIN_ANGLE_DEG)
        side_max = math.radians(LIDAR_SIDE_MAX_ANGLE_DEG)
        rear_plane = -REAR_AXLE_BEHIND_LIDAR_M
        front_hits: list[tuple[float, float]] = []
        left_hits: list[tuple[float, float]] = []
        right_hits: list[tuple[float, float]] = []

        for i, r in enumerate(lidar_msg.ranges):
            if not math.isfinite(r):
                continue
            if not (LIDAR_MIN_RANGE_M < r < LIDAR_MAX_RANGE_M):
                continue
            angle = self._normalize_scan_angle(angle_min + i * angle_increment)
            if self._lidar_hit_forward_x(angle, r) < rear_plane:
                continue
            abs_angle = abs(angle)

            if abs_angle <= front_half:
                front_hits.append((angle, r))
            elif side_min <= angle <= side_max:
                left_hits.append((angle, r))
            elif -side_max <= angle <= -side_min:
                right_hits.append((angle, r))

        return front_hits, left_hits, right_hits

    @staticmethod
    def _lidar_hit_forward_x(angle: float, range_m: float) -> float:
        """Forward component of a scan hit in lidar/base frame (+x = ahead)."""
        return range_m * math.cos(angle)

    def get_lidar_sector_minimums(self):
        """
        Minimum range (m) per forward sector from raw /scan using bearing angles.

        Hits behind the rear-axle plane (x < -REAR_AXLE_BEHIND_LIDAR_M) are excluded
        here; they belong in get_lidar_rear_minimum() for backward motion only.

        Sectors (angle 0 = lidar forward):
          front : |angle| <= LIDAR_FRONT_HALF_ANGLE_DEG
          left  : +LIDAR_SIDE_MIN..+LIDAR_SIDE_MAX deg
          right : -LIDAR_SIDE_MAX..-LIDAR_SIDE_MIN deg

        Returns (front, left, right); each may be None if no valid returns.
        """
        lidar_msg = self.ros_communicator.get_latest_lidar()
        if lidar_msg is None:
            return None, None, None

        front_hits, left_hits, right_hits = self._collect_lidar_forward_hits(lidar_msg)
        lf, _ = self._lidar_sector_range_from_hits(front_hits)
        ll, _ = self._lidar_sector_range_from_hits(left_hits)
        lr, _ = self._lidar_sector_range_from_hits(right_hits)
        return lf, ll, lr

    def get_lidar_sector_closest_hits(self):
        """
        Debug helper: per forward sector, report robust min plus raw closest hit.

        Returns dict with keys front/left/right, each either None or:
          {robust_m, raw_min_m, raw_min_angle_deg, hit_count}
        """
        lidar_msg = self.ros_communicator.get_latest_lidar()
        if lidar_msg is None:
            return {"front": None, "left": None, "right": None}

        front_hits, left_hits, right_hits = self._collect_lidar_forward_hits(lidar_msg)
        buckets = {"front": front_hits, "left": left_hits, "right": right_hits}

        out: dict[str, dict | None] = {}
        for name, hits in buckets.items():
            if not hits:
                out[name] = None
                continue
            raw_min_r, raw_min_deg = min(
                ((r, math.degrees(a)) for a, r in hits),
                key=lambda t: t[0],
            )
            robust, close_span = self._lidar_sector_range_from_hits(hits)
            out[name] = {
                "robust_m": robust,
                "raw_min_m": raw_min_r,
                "raw_min_angle_deg": raw_min_deg,
                "hit_count": len(hits),
                "close_span_deg": close_span,
            }
        return out

    def get_lidar_rear_minimum(self):
        """
        Minimum range (m) behind the rear-axle plane — for backward motion only.

        Includes LiDAR returns with forward x < -REAR_AXLE_BEHIND_LIDAR_M and
        rear-arc angles (|angle| >= LIDAR_REAR_MIN_ANGLE_DEG).
        """
        lidar_msg = self.ros_communicator.get_latest_lidar()
        if lidar_msg is None:
            return None

        angle_min = lidar_msg.angle_min
        angle_increment = lidar_msg.angle_increment
        rear_plane = -REAR_AXLE_BEHIND_LIDAR_M
        rear_angle = math.radians(LIDAR_REAR_MIN_ANGLE_DEG)
        rear_vals: list[float] = []

        for i, r in enumerate(lidar_msg.ranges):
            if not math.isfinite(r):
                continue
            angle = self._normalize_scan_angle(angle_min + i * angle_increment)
            x_fwd = self._lidar_hit_forward_x(angle, r)
            if x_fwd < rear_plane or abs(angle) >= rear_angle:
                rear_vals.append(r)

        return self._robust_sector_distance(rear_vals)

    def get_processed_lidar(self):
        lidar_msg = self.ros_communicator.get_latest_lidar()
        if lidar_msg is None:
            return None
        angle_min = lidar_msg.angle_min
        angle_increment = lidar_msg.angle_increment
        ranges_180 = []
        all_ranges = lidar_msg.ranges
        for i in range(len(all_ranges)):
            if i % LIDAR_PER_SECTOR == 0:  # handle the amount of lidar.
                ranges_180.append(all_ranges[i])
        combined_lidar_data = (
            self._pick_downsampled_ranges(ranges_180, FRONT_LIDAR_INDICES)
            + self._pick_downsampled_ranges(ranges_180, LEFT_LIDAR_INDICES)
            + self._pick_downsampled_ranges(ranges_180, RIGHT_LIDAR_INDICES)
        )
        return combined_lidar_data

    import time

    def get_processed_mediapipe_data(self):
        mediapipe_data_msg = self.ros_communicator.get_latest_mediapipe_data()

        # 檢查是否接收到資料，並從中提取座標
        if mediapipe_data_msg is not None:
            # 將 x, y, z 座標放入列表
            coordinates_list = [
                mediapipe_data_msg.x,
                mediapipe_data_msg.y,
                mediapipe_data_msg.z,
            ]
            return coordinates_list
        else:
            # 如果資料為 None，返回空列表或其他指示資料無效的值
            return []

    def get_processed_yolo_detection_position(self):

        yolo_detection_position_msg = (
            self.ros_communicator.get_latest_yolo_detection_position()
        )
        if yolo_detection_position_msg is not None:
            return [
                yolo_detection_position_msg.point.x,
                yolo_detection_position_msg.point.y,
                yolo_detection_position_msg.point.z,
            ]

        else:
            return None

    def get_processed_yolo_detection_offset(self):
        yolo_detection_offset_msg = (
            self.ros_communicator.get_latest_yolo_detection_offset()
        )
        if yolo_detection_offset_msg is not None:
            return [
                yolo_detection_offset_msg.point.x,
                yolo_detection_offset_msg.point.y,
                yolo_detection_offset_msg.point.z,
            ]
        else:
            return None

    def get_processed_received_global_plan(self):
        received_global_plan_msg = (
            self.ros_communicator.get_latest_received_global_plan()
        )
        if received_global_plan_msg is None:
            return None, None
        path_length = len(received_global_plan_msg.poses)
        orientation_points = []
        coordinates = []
        if path_length > 0:
            last_recorded_point = received_global_plan_msg.poses[0].pose.position
            orientation_points.append(
                (
                    received_global_plan_msg.poses[0].pose.orientation.z,
                    received_global_plan_msg.poses[0].pose.orientation.w,
                )
            )
            coordinates.append(
                (
                    received_global_plan_msg.poses[0].pose.position.x,
                    received_global_plan_msg.poses[0].pose.position.y,
                )
            )
            for i in range(1, path_length):
                current_point = received_global_plan_msg.poses[i].pose.position
                distance = math.sqrt(
                    (current_point.x - last_recorded_point.x) ** 2
                    + (current_point.y - last_recorded_point.y) ** 2
                )
                if distance >= 0.1:
                    orientation_points.append(
                        (
                            received_global_plan_msg.poses[i].pose.orientation.z,
                            received_global_plan_msg.poses[i].pose.orientation.w,
                        )
                    )
                    coordinates.append((current_point.x, current_point.y))
                    last_recorded_point = current_point
        return orientation_points, coordinates

    def get_processed_received_global_plan_no_dynamic(self):
        received_global_plan_msg = (
            self.ros_communicator.get_latest_received_global_plan()
        )

        if not received_global_plan_msg or not received_global_plan_msg.poses:
            print("沒接收到路徑")
            return None

        goal_position = self.ros_communicator.get_latest_goal()
        if goal_position is None:
            print("未設定 goal_pose")
            return None

        last_point = received_global_plan_msg.poses[-1].pose.position
        last_x, last_y = last_point.x, last_point.y
        goal_x, goal_y = goal_position[:2]

        distance_to_goal = math.sqrt((last_x - goal_x) ** 2 + (last_y - goal_y) ** 2)

        # 如果該條路徑的末端有靠近終點就當成是成功的路徑
        if distance_to_goal < 0.2:
            self.ros_communicator.publish_confirmed_initial_plan(
                received_global_plan_msg
            )
            return received_global_plan_msg
        else:
            return None
