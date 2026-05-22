"""
YOLO semantic/instance segmentation node for HW4 (road / bridge).

Uses segmentation.pt (YOLO-seg weights). Run separately from yolo_node:
  ros2 run yolo_example_pkg seg_node

Foxglove: subscribe to /yolo/segmentation/compressed

Requires ultralytics recent enough for your checkpoint (e.g. YOLO26-seg).

Do not publish /camera/x_multi_depth_values here so that yolo_node + seg_node
can run together without competing on that topic (navigation uses yolo_node).
"""

import os

import cv2
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO


class YoloSegmentationNode(Node):
    def __init__(self):
        super().__init__("yolo_segmentation_node")

        self.bridge = CvBridge()

        model_path = os.path.join(
            get_package_share_directory("yolo_example_pkg"),
            "models",
            "segmentation.pt",
        )

        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Loading seg model: {model_path} device={device}")
        self.model = YOLO(model_path)
        self.model.to(device)

        self.image_sub = self.create_subscription(
            CompressedImage,
            "/camera/image/compressed",
            self.image_callback,
            1,
        )

        self.image_pub = self.create_publisher(
            CompressedImage,
            "/yolo/segmentation/compressed",
            10,
        )

        self.conf_threshold = 0.5

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        try:
            results = self.model(cv_image, conf=self.conf_threshold, verbose=False)
        except Exception as e:
            self.get_logger().error(f"YOLO segmentation failed: {e}")
            return

        plotted = results[0].plot()
        self._publish_image(plotted)

    def _publish_image(self, image):
        try:
            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(image)
            self.image_pub.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f"Could not publish image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
