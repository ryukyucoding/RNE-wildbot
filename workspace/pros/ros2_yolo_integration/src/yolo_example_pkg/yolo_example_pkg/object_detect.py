import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray, String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory
import torch

# Unity/rosbridge 轉發的相機 topic 使用 TRANSIENT_LOCAL，訂閱端需一致才能收到影像
CAMERA_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

COLOR_MAP = {
    "bear": (0, 60, 200),
    "knob": (200, 200, 0),
    "bridge": (0, 140, 255),
    "road": (180, 120, 0),
}


class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__("yolo_detection_node")

        self.bridge = CvBridge()
        self.latest_depth_image_raw = None
        self.latest_depth_image_compressed = None
        self.target_label = ""

        model_name = os.environ.get("YOLO_MODEL", "detection.pt")
        model_path = os.path.join(
            get_package_share_directory("yolo_example_pkg"), "models", model_name
        )

        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {device}")
        self.model = YOLO(model_path)
        self.model.to(device)
        self.get_logger().info(f"Model: {model_name}, classes: {dict(self.model.names)}")

        self.depth_sub_raw = self.create_subscription(
            Image, "/camera/depth/image_raw", self.depth_callback_raw, 1
        )
        self.target_label_sub = self.create_subscription(
            String, "/target_label", self.target_label_callback, 10
        )

        self.image_pub = self.create_publisher(
            CompressedImage, "/yolo/detection/compressed", 10
        )
        self.target_pub = self.create_publisher(
            Float32MultiArray, "/yolo/target_info", 10
        )
        self.x_multi_depth_pub = self.create_publisher(
            Float32MultiArray, "/camera/x_multi_depth_values", 10
        )

        self.conf_threshold = float(os.environ.get("YOLO_CONF", "0.5"))
        self.x_num_splits = 20
        self._got_image = False

        self.image_sub = self.create_subscription(
            CompressedImage, "/camera/image/compressed", self.image_callback, CAMERA_QOS
        )
        self.depth_sub_compressed = self.create_subscription(
            CompressedImage,
            "/camera/depth/compressed",
            self.depth_callback_compressed,
            CAMERA_QOS,
        )
        self.get_logger().info(
            "Subscribed to /camera/image/compressed (TRANSIENT_LOCAL QoS). "
            "Waiting for Unity camera..."
        )

    def target_label_callback(self, msg):
        self.target_label = (msg.data or "").strip()

    def depth_callback_raw(self, msg):
        try:
            self.latest_depth_image_raw = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
        except Exception as e:
            self.get_logger().error(f"Could not convert raw depth image: {e}")

    def depth_callback_compressed(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            depth_img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            if depth_img is not None:
                self.latest_depth_image_compressed = depth_img
        except Exception as e:
            self.get_logger().error(f"Could not convert compressed depth image: {e}")

    def image_callback(self, msg):
        if not self._got_image:
            self._got_image = True
            self.get_logger().info("Receiving /camera/image/compressed — YOLO pipeline active.")

        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        try:
            results = self.model(cv_image, conf=self.conf_threshold, verbose=False)
        except Exception as e:
            self.get_logger().error(f"Error during YOLO detection: {e}")
            return

        processed_image, target_info = self.draw_bounding_boxes(cv_image, results)
        self.publish_x_multi_depths(processed_image)
        self.publish_image(processed_image)
        self.publish_target_info(*target_info)

    def pick_target_box(self, boxes):
        best = None
        best_idx = None
        best_conf = -1.0
        for i, box in enumerate(boxes):
            class_name = self.model.names[int(box.cls[0])]
            if self.target_label and class_name != self.target_label:
                continue
            conf = float(box.conf[0])
            if conf > best_conf:
                best_conf = conf
                best = box
                best_idx = i
        return best, best_idx

    def draw_bounding_boxes(self, image, results):
        height, width = image.shape[:2]
        cx_center = width // 2
        out = image.copy()
        found, distance, delta_x = 0.0, 0.0, 0.0
        target_box = None

        for result in results:
            if result.boxes is None:
                continue
            target_box, target_idx = self.pick_target_box(result.boxes)

            if result.masks is not None:
                masks = result.masks.data.cpu().numpy()
                boxes = result.boxes
                for i, mask in enumerate(masks):
                    class_id = int(boxes.cls[i])
                    class_name = self.model.names[class_id]
                    mask_resized = cv2.resize(mask, (width, height))
                    mask_bool = mask_resized > 0.5
                    color = COLOR_MAP.get(class_name, (128, 128, 128))
                    mask_colored = np.zeros_like(out)
                    mask_colored[mask_bool] = color
                    out = cv2.addWeighted(out, 1, mask_colored, 0.35, 0)

            for i, box in enumerate(result.boxes):
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                color = COLOR_MAP.get(class_name, (0, 255, 0))
                thickness = 3 if i == target_idx else 2
                cv2.rectangle(out, (x1, y1), (x2, y2), color, thickness)
                label = f"{class_name} {conf:.2f}"
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(out, (x1, y1 - th - 8), (x1 + tw + 4, y1), color, -1)
                cv2.putText(
                    out, label, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2,
                )

        if target_box is not None:
            x1, y1, x2, y2 = map(int, target_box.xyxy[0])
            tcx = (x1 + x2) // 2
            tcy = (y1 + y2) // 2
            depth_m = self.get_depth_at(tcx, tcy)
            found = 1.0
            delta_x = float(tcx - cx_center)
            if depth_m < 0:
                distance = 0.0
            elif depth_m < 0.4:
                distance = -1.0
            else:
                distance = depth_m

        return out, (found, distance, delta_x)

    def get_depth_at(self, x, y):
        depth_image = (
            self.latest_depth_image_raw
            if self.latest_depth_image_raw is not None
            else self.latest_depth_image_compressed
        )
        if depth_image is None:
            return -1.0
        if len(depth_image.shape) == 3:
            depth_image = depth_image[:, :, 0]
        try:
            depth_value = float(depth_image[y, x])
            if depth_value < 0.0001 or depth_value == 0.0:
                return -1.0
            return depth_value / 1000.0
        except IndexError:
            return -1.0

    def publish_image(self, image):
        try:
            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(image)
            self.image_pub.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f"Could not publish image: {e}")

    def publish_target_info(self, found, distance, delta_x):
        msg = Float32MultiArray()
        msg.data = [float(found), float(distance), float(delta_x)]
        self.target_pub.publish(msg)

    def publish_x_multi_depths(self, image):
        height, width = image.shape[:2]
        cy_center = height // 2
        segment_length = width // self.x_num_splits
        points = [(i * segment_length, cy_center) for i in range(self.x_num_splits + 1)]
        depth_values = [self.get_depth_at(x, cy_center) for x, _ in points]
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
