import math
import os
from pathlib import Path

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO
from visualization_msgs.msg import Marker


class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__("yolo_detection_node")

        self.declare_parameter("target_class", "")
        self.declare_parameter("camera_optical_frame", "camera_optical_frame")
        self.declare_parameter("publish_target_marker", True)
        self.declare_parameter("depth_units_per_meter", 1000.0)
        self.declare_parameter("publish_sector_min_depth", True)
        self.declare_parameter("sector_depth_min_m", 0.40)
        self.declare_parameter("sector_depth_max_m", 12.0)
        self.declare_parameter("weights_path", "")
        # nearest: 多隻目標時選深度最近（最適合直接接近）; confidence: 選信心最高
        self.declare_parameter("target_select_mode", "nearest")

        self.target_class = (
            self.get_parameter("target_class").get_parameter_value().string_value.strip()
        )
        self.optical_frame = (
            self.get_parameter("camera_optical_frame")
            .get_parameter_value()
            .string_value
        )
        self.publish_target_marker = (
            self.get_parameter("publish_target_marker").get_parameter_value().bool_value
        )
        self._depth_units_per_meter = (
            self.get_parameter("depth_units_per_meter")
            .get_parameter_value()
            .double_value
        )
        if self._depth_units_per_meter < 1e-6:
            self._depth_units_per_meter = 1000.0
        self.publish_sector_min_depth = (
            self.get_parameter("publish_sector_min_depth")
            .get_parameter_value()
            .bool_value
        )
        self._sector_depth_min_m = (
            self.get_parameter("sector_depth_min_m").get_parameter_value().double_value
        )
        self._sector_depth_max_m = (
            self.get_parameter("sector_depth_max_m").get_parameter_value().double_value
        )
        self.target_select_mode = (
            self.get_parameter("target_select_mode")
            .get_parameter_value()
            .string_value.strip()
            .lower()
        )
        if self.target_select_mode not in ("nearest", "confidence"):
            self.target_select_mode = "nearest"

        self.bridge = CvBridge()
        self._lock_cx = None        # 鎖定目標的畫面 x（像素）
        self._lock_cy = None        # 鎖定目標的畫面 y（像素）
        self._lock_depth = None     # 鎖定時的深度
        self._lock_init_depth = None  # 第一次鎖定時的深度（不隨靠近更新）
        self._lock_lost_since = None
        self._lock_reselect_delay = 2.0

        self.latest_depth_image_raw = None
        self.latest_depth_image_compressed = None
        self._camera_info = None
        self._fx = 554.0
        self._fy = 554.0
        self._cx = 320.0
        self._cy = 240.0

        # Must be a detect checkpoint (.pt); segmentation.pt raises wrong-task errors here.
        model_path = self._resolve_weights_path()

        device = "cuda" if torch.cuda.is_available() else "cpu"
        print("Using device : ", device)
        self.model = YOLO(model_path)
        self.model.to(device)

        # 訂閱影像 Topic
        self.image_sub = self.create_subscription(
            CompressedImage, "/camera/image/compressed", self.image_callback, 1
        )

        # 訂閱 **無壓縮** 深度圖 Topic
        self.depth_sub_raw = self.create_subscription(
            Image, "/camera/depth/image_raw", self.depth_callback_raw, 1
        )

        # 訂閱 **壓縮** 深度圖 Topic
        self.depth_sub_compressed = self.create_subscription(
            CompressedImage,
            "/camera/depth/compressed",
            self.depth_callback_compressed,
            1,
        )

        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/image/camera_info",
            self._camera_info_cb,
            1,
        )

        # 發佈處理後的影像 Topic
        self.image_pub = self.create_publisher(
            CompressedImage, "/yolo/detection/compressed", 10
        )

        # 發布 目標檢測數據 (是否找到目標 + 距離)
        self.target_pub = self.create_publisher(
            Float32MultiArray, "/yolo/target_info", 10
        )

        self.x_multi_depth_pub = self.create_publisher(
            Float32MultiArray, "/camera/x_multi_depth_values", 10
        )

        self.sector_depth_pub = self.create_publisher(
            Float32MultiArray, "/obstacle/sector_min_depth", 10
        )

        self.marker_pub = self.create_publisher(Marker, "/yolo/target_marker", 10)

        # 設定要過濾標籤：若為空集合 set()，則不過濾，會畫出模型回傳的所有類別。
        # （先前設 {"tennis"} 會濾掉 HW4 的 bear/knob；COCO 網球多是 "sports ball" 而非 tennis）
        self.allowed_labels = set()

        # 設定 YOLO 可信度閾值
        self.conf_threshold = 0.5  # 可以修改這個值來調整可信度

        # 相機畫面中央高度上切成 n 個等距水平點。
        self.x_num_splits = 20

        self.get_logger().info(
            f"YOLO detect: weights={model_path}, target_class='{self.target_class or '(any)'}', "
            f"target_select_mode={self.target_select_mode}, "
            f"publish_target_marker={self.publish_target_marker}, "
            f"optical_frame={self.optical_frame}, depth_units_per_m={self._depth_units_per_meter}"
        )

    def _resolve_weights_path(self) -> str:
        override = (
            self.get_parameter("weights_path").get_parameter_value().string_value.strip()
        )
        candidates: list[Path] = []
        if override:
            candidates.append(Path(override).expanduser())

        preferred_names = ("detection.pt", "object.pt")

        try:
            share = Path(get_package_share_directory("yolo_example_pkg"))
            sm = share / "models"
            for name in preferred_names:
                candidates.append(sm / name)
            if sm.is_dir():
                candidates.extend(sorted(sm.glob("*.pt")))
        except PackageNotFoundError:
            pass

        src_models = Path(__file__).resolve().parent.parent / "models"
        for name in preferred_names:
            candidates.append(src_models / name)
        if src_models.is_dir():
            candidates.extend(sorted(src_models.glob("*.pt")))

        seen: set[str] = set()
        for p in candidates:
            try:
                key = str(p.resolve())
            except Exception:
                key = str(p)
            if key in seen:
                continue
            seen.add(key)
            if p.is_file():
                self.get_logger().info(f"YOLO weights: {p}")
                return str(p)

        hint = (
            "找不到 YOLO .pt。請將檔放在 "
            "yolo_example_pkg/models/detection.pt 或 object.pt（或 models 下任一 .pt）"
            "後執行 colcon build；或使用 "
            "ros2 run yolo_example_pkg yolo_node --ros-args -p weights_path:=/絕對路徑/model.pt"
        )
        self.get_logger().fatal(hint)
        raise FileNotFoundError(hint)

    def _camera_info_cb(self, msg: CameraInfo):
        self._camera_info = msg
        self._fx = float(msg.k[0])
        self._fy = float(msg.k[4])
        self._cx = float(msg.k[2])
        self._cy = float(msg.k[5])

    def depth_callback_raw(self, msg):
        """接收 **無壓縮** 深度圖"""
        try:
            self.latest_depth_image_raw = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
            if self.publish_sector_min_depth:
                self.publish_sector_min_depths()
        except Exception as e:
            self.get_logger().error(f"Could not convert raw depth image: {e}")

    def depth_callback_compressed(self, msg):
        """接收 **壓縮** 深度圖（當無壓縮深度圖不可用時使用）"""
        try:
            # 自行強制使用 cv2.IMREAD_UNCHANGED 解碼，避開 cv_bridge 的潛在雷區
            np_arr = np.frombuffer(msg.data, np.uint8)
            depth_img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            if depth_img is not None:
                self.latest_depth_image_compressed = depth_img
                if self.publish_sector_min_depth:
                    self.publish_sector_min_depths()
        except Exception as e:
            self.get_logger().error(f"Could not convert compressed depth image: {e}")

    def image_callback(self, msg):
        """接收影像並進行物體檢測"""
        # 將 ROS 影像消息轉換為 OpenCV 格式
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        # 使用 YOLO 模型檢測物體
        try:
            results = self.model(cv_image, conf=self.conf_threshold, verbose=False)
        except Exception as e:
            self.get_logger().error(f"Error during YOLO detection: {e}")
            return

        # 繪製 Bounding Box
        processed_image = self.draw_bounding_boxes(cv_image, results)

        # 取得影像中心深度並發布
        self.publish_x_multi_depths(processed_image)
        if self.publish_sector_min_depth:
            self.publish_sector_min_depths()

        # 發佈處理後的影像
        self.publish_image(processed_image)

    def draw_cross(self, image):
        # 回傳繪製十字架的影像和畫面正中間的像素座標
        height, width = image.shape[:2]
        cx_center = width // 2
        cy_center = height // 2
        # 繪製橫線
        cv2.line(image, (0, cy_center), (width, cy_center), (0, 0, 255), 2)

        # 繪製直線
        cv2.line(
            image,
            (cx_center, cy_center - 10),
            (cx_center, cy_center + 10),
            (0, 0, 255),
            2,
        )

        cv2.line(
            image,
            (cx_center, cy_center - 10),
            (cx_center, cy_center + 10),
            (0, 0, 255),
            2,
        )

        # 計算橫線上的 n 個等分點
        segment_length = width // self.x_num_splits
        points = [
            (i * segment_length, cy_center) for i in range(self.x_num_splits + 1)
        ]  # 11 個點表示 10 段區間的端點

        # 在每個等分點繪製垂直的短黑線
        for x, y in points:
            cv2.line(image, (x, y - 10), (x, y + 10), (0, 0, 0), 2)  # 黑色垂直線

        return image, points

    def draw_bounding_boxes(self, image, results):
        """繪製 bbox；target_info 與 target_marker 僅對「優先目標」類別（見 target_class）。"""
        found_target = 0
        target_distance = 0.0
        delta_x = 0.0
        image, _points = self.draw_cross(image)
        height, width = image.shape[:2]
        cx_center = width // 2

        candidates = []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf)
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]

                if self.allowed_labels and class_name not in self.allowed_labels:
                    continue

                if self.target_class and class_name != self.target_class:
                    color = (180, 180, 0)
                    cv2.rectangle(image, (x1, y1), (x2, y2), color, 1)
                    continue

                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                candidates.append((conf, x1, y1, x2, y2, class_name, cx, cy))

        scored = []
        for conf, x1, y1, x2, y2, class_name, cx, cy in candidates:
            depth_value = self.get_depth_median(cx, cy)
            scored.append((conf, x1, y1, x2, y2, class_name, cx, cy, depth_value))

        primary = self._select_primary_target(scored)
        primary_key = None

        for conf, x1, y1, x2, y2, class_name, cx, cy, depth_value in scored:
            is_primary = primary is not None and (cx, cy) == (primary[6], primary[7])
            if is_primary:
                primary_key = (cx, cy)
            depth_text = f"{depth_value:.2f}m" if depth_value > 0 else "N/A"
            color = (0, 140, 255) if is_primary else (0, 255, 0)
            thickness = 3 if is_primary else 1
            cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)
            tag = "TARGET " if is_primary else ""
            label = f"{tag}{class_name} {conf:.2f} Depth: {depth_text}"
            cv2.putText(
                image,
                label,
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
            )

        if primary is not None:
            _c, _x1, _y1, _x2, _y2, class_name, cx, cy, depth_value = primary
            if depth_value > 0:
                found_target = 1
                target_distance = depth_value
                delta_x = float(cx - cx_center)
                self._lock_cx = float(cx)
                self._lock_cy = float(cy)
                self._lock_depth = float(depth_value)
                # 只在第一次鎖定時記錄初始深度，之後不更新
                if self._lock_init_depth is None:
                    self._lock_init_depth = float(depth_value)
                self._lock_lost_since = None

                if self.publish_target_marker:
                    self._publish_bear_marker(cx, cy, depth_value, class_name)
        else:
            import time as _time
            if self._lock_lost_since is None:
                self._lock_lost_since = _time.monotonic()
            # 鎖定消失超過 reselect_delay 才清除，避免旋轉時瞬間換目標
            if _time.monotonic() - self._lock_lost_since > self._lock_reselect_delay:
                self._lock_cx = None
                self._lock_cy = None
                self._lock_depth = None
                self._lock_init_depth = None
                self._lock_lost_since = None

        self.publish_target_info(found_target, target_distance, delta_x)
        return image

    def _select_primary_target(self, scored):
        """多隻熊時選一隻作為導航目標。

        鎖定策略：
        - 未鎖定：選畫面中最近的熊（depth 最小）
        - 已鎖定：用「初始鎖定深度」比對，找深度最接近的那隻繼續跟
          （不用當前深度，避免靠近後鎖定漂移到另一隻）
        - 若所有候選與初始深度相差 > 1.0m，代表原本那隻消失，重新選最近
        """
        valid = [row for row in scored if row[8] > 0.05]
        if not valid:
            return None

        if self.target_select_mode == "confidence":
            return max(valid, key=lambda t: t[0])

        # 已鎖定：用初始深度（不隨靠近更新）來識別同一隻熊
        ref_depth = self._lock_init_depth if self._lock_init_depth is not None else self._lock_depth
        if ref_depth is not None:
            by_match = sorted(valid, key=lambda t: abs(float(t[8]) - ref_depth))
            best = by_match[0]
            # 容許深度差在 1.0m 以內（從 2m 靠近到 0.4m 的過程中一定涵蓋）
            if abs(float(best[8]) - ref_depth) < 1.0:
                return best

        # 未鎖定或鎖定目標消失：選畫面中最近的
        return min(valid, key=lambda t: t[8])

    def get_depth_median(self, cx, cy, radius=4):
        vals = []
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                d = self.get_depth_at(cx + dx, cy + dy)
                if d > 0.05:
                    vals.append(d)
        if not vals:
            return -1.0
        return float(np.median(vals))

    def _publish_bear_marker(self, u, v, depth_m, label_name):
        """在 camera optical frame 發布球體 Marker（供手臂 IK 使用 TF 轉到 arm_ik_base）。"""
        try:
            fx, fy, cx0, cy0 = self._fx, self._fy, self._cx, self._cy
            if fx < 1e-6 or fy < 1e-6:
                return
            X = (float(u) - cx0) * depth_m / fx
            Y = (float(v) - cy0) * depth_m / fy
            Z = depth_m
        except Exception as e:
            self.get_logger().warning(f"marker projection skip: {e}")
            return

        m = Marker()
        m.header.frame_id = self.optical_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "yolo_target"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = Point(x=float(X), y=float(Y), z=float(Z))
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.12
        m.color.a = 1.0
        m.color.r = 1.0
        m.color.g = 0.6
        m.color.b = 0.1
        self.marker_pub.publish(m)

    def _depth_image_meters(self):
        depth_image = (
            self.latest_depth_image_raw
            if self.latest_depth_image_raw is not None
            else self.latest_depth_image_compressed
        )
        if depth_image is None:
            return None
        if len(depth_image.shape) == 3:
            depth_image = depth_image[:, :, 0]
        scale = float(self._depth_units_per_meter)
        depth_m = depth_image.astype(np.float32) / scale
        depth_m[(depth_image <= 0) | ~np.isfinite(depth_m)] = -1.0
        return depth_m

    def _sector_patch_min(self, depth_m: np.ndarray, y0: int, y1: int, x0: int, x1: int) -> float:
        patch = depth_m[y0:y1, x0:x1]
        valid = patch[
            (patch > 0.0)
            & (patch != -1.0)
            & np.isfinite(patch)
            & (patch >= self._sector_depth_min_m)
            & (patch <= self._sector_depth_max_m)
        ]
        if valid.size == 0:
            return -1.0
        return float(np.min(valid))

    def publish_sector_min_depths(self):
        """
        發布 5 方向最小深度（公尺）:
        [front, front_left, left, front_right, right]
        """
        depth_m = self._depth_image_meters()
        if depth_m is None:
            return

        h, w = depth_m.shape[:2]
        y0 = int(h * 0.35)
        y1 = h
        if y1 <= y0:
            y0, y1 = 0, h

        col_w = max(1, w // 5)
        bands = [
            (0, col_w),
            (col_w, 2 * col_w),
            (2 * col_w, 3 * col_w),
            (3 * col_w, 4 * col_w),
            (4 * col_w, w),
        ]

        def band_min(col_idx: int) -> float:
            x0, x1 = bands[col_idx]
            return self._sector_patch_min(depth_m, y0, y1, x0, x1)

        front = band_min(2)
        front_left = band_min(1)
        left = band_min(0)
        front_right = band_min(3)
        right = band_min(4)

        msg = Float32MultiArray()
        msg.data = [
            float(front),
            float(front_left),
            float(left),
            float(front_right),
            float(right),
        ]
        self.sector_depth_pub.publish(msg)

    def get_depth_at(self, x, y):
        """
        取得指定像素的深度值，轉換為米 (m)
        若深度出問題，回傳 -1
        """
        # **優先使用無壓縮的深度圖**
        depth_image = (
            self.latest_depth_image_raw
            if self.latest_depth_image_raw is not None
            else self.latest_depth_image_compressed
        )

        if depth_image is None:
            return -1.0

        # 如果深度影像為三通道，那只取第一個數值
        if len(depth_image.shape) == 3:
            depth_image = depth_image[:, :, 0]

        try:
            depth_value = depth_image[y, x]
            if depth_value < 0.0001 or depth_value == 0.0:  # 無效深度
                return -1.0
            return float(depth_value) / float(self._depth_units_per_meter)
        except IndexError:
            return -1.0

    def publish_image(self, image):
        """將處理後的影像轉換並發佈到 ROS"""
        try:
            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(image)
            self.image_pub.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f"Could not publish image: {e}")

    def publish_target_info(self, found, distance, delta_x):
        """發佈目標資訊 (找到目標, 距離)"""
        msg = Float32MultiArray()
        msg.data = [float(found), float(distance), float(delta_x)]
        self.target_pub.publish(msg)

    def publish_x_multi_depths(self, image):
        """
        取得畫面 n 個等分點的深度並發布
        """
        height, width = image.shape[:2]
        cy_center = height // 2  # 固定 Y 座標在畫面中心
        segment_length = width // self.x_num_splits

        # 計算 10 個等分點的 X 座標
        points = [(i * segment_length, cy_center) for i in range(self.x_num_splits)]

        # 取得每個等分點的深度值
        depth_values = [self.get_depth_at(x, cy_center) for x, _ in points]

        # 以 Float32MultiArray 發布
        depth_msg = Float32MultiArray()
        depth_msg.data = depth_values
        self.x_multi_depth_pub.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
