import os
import cv2
import numpy as np
import roslibpy
from ultralytics import YOLO
import base64

# open_door / HW4 detection model: classes bear, knob
DEFAULT_MODEL = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "../src/yolo_example_pkg/models/detection_knob.pt")
)

COLOR_MAP = {
    "bear": (0, 60, 200),
    "knob": (200, 200, 0),
}


class YOLOProcessor:
    def __init__(self, model_path, ros_host, ros_port, conf_threshold=0.5):
        import torch

        device = (
            "mps"
            if torch.backends.mps.is_available()
            else ("cuda" if torch.cuda.is_available() else "cpu")
        )
        self.model = YOLO(model_path)
        self.model.to(device)
        self.conf_threshold = conf_threshold
        self.target_label = ""
        self.latest_depth = None
        print(f"Using device: {device}")
        print(f"Model: {os.path.basename(model_path)}")
        print(f"Classes: {dict(self.model.names)}")

        self.client = roslibpy.Ros(host=ros_host, port=ros_port)
        self.client.run()

        if self.client.is_connected:
            print(f"Connected to rosbridge at ws://{ros_host}:{ros_port}")

        self.image_listener = roslibpy.Topic(
            self.client, "/camera/image/compressed", "sensor_msgs/msg/CompressedImage"
        )
        self.depth_listener = roslibpy.Topic(
            self.client, "/camera/depth/compressed", "sensor_msgs/msg/CompressedImage"
        )
        self.target_label_listener = roslibpy.Topic(
            self.client, "/target_label", "std_msgs/msg/String"
        )
        self.yolo_publisher = roslibpy.Topic(
            self.client, "/yolo/detection/compressed", "sensor_msgs/msg/CompressedImage"
        )
        self.target_info_publisher = roslibpy.Topic(
            self.client, "/yolo/target_info", "std_msgs/msg/Float32MultiArray"
        )

    def on_target_label(self, msg):
        self.target_label = (msg.get("data") or "").strip()

    def on_depth(self, data):
        compressed = data.get("data")
        if isinstance(compressed, str):
            compressed = base64.b64decode(compressed)
        img_data = np.frombuffer(compressed, np.uint8)
        depth_img = cv2.imdecode(img_data, cv2.IMREAD_UNCHANGED)
        if depth_img is not None:
            self.latest_depth = depth_img

    def get_depth_at(self, x, y):
        if self.latest_depth is None:
            return -1.0
        depth_image = self.latest_depth
        if len(depth_image.shape) == 3:
            depth_image = depth_image[:, :, 0]
        h, w = depth_image.shape[:2]
        if not (0 <= x < w and 0 <= y < h):
            return -1.0
        depth_value = float(depth_image[y, x])
        if depth_value < 0.0001 or depth_value == 0.0:
            return -1.0
        return depth_value / 1000.0

    def pick_target_box(self, boxes):
        """Return (best_box, index) matching target_label, or (None, None)."""
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

    def process_image(self, data):
        compressed_img = data["data"]
        if isinstance(compressed_img, str):
            compressed_img = base64.b64decode(compressed_img)

        img_data = np.frombuffer(compressed_img, np.uint8)
        img_bgr = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
        if img_bgr is None:
            print("Failed to decode image.")
            return

        img_with_boxes, target_info = self.detect_objects(img_bgr)

        _, compressed_out = cv2.imencode(".jpg", img_with_boxes)
        base64_data = base64.b64encode(compressed_out.tobytes()).decode("utf-8")
        self.yolo_publisher.publish(
            roslibpy.Message({"format": "jpeg", "data": base64_data})
        )
        self.target_info_publisher.publish(
            roslibpy.Message({"data": [float(v) for v in target_info]})
        )

    def detect_objects(self, img):
        h, w = img.shape[:2]
        cx_center = w // 2
        results = self.model(img, conf=self.conf_threshold, verbose=False)

        found, distance, delta_x = 0.0, 0.0, 0.0
        target_box = None

        for result in results:
            if result.boxes is None:
                continue
            target_box, target_idx = self.pick_target_box(result.boxes)
            for i, box in enumerate(result.boxes):
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                class_name = self.model.names[int(box.cls[0])]
                conf = float(box.conf[0])
                color = COLOR_MAP.get(class_name, (0, 255, 0))
                thickness = 3 if i == target_idx else 2
                cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)
                label = f"{class_name}: {conf:.2f}"
                cv2.putText(
                    img, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
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

        return img, (found, distance, delta_x)

    def start_processing(self):
        self.target_label_listener.subscribe(self.on_target_label)
        self.depth_listener.subscribe(self.on_depth)
        self.image_listener.subscribe(self.process_image)
        print("Waiting for images... (set /target_label to 'knob' for door_open)")

    def stop_processing(self):
        self.image_listener.unsubscribe()
        self.depth_listener.unsubscribe()
        self.target_label_listener.unsubscribe()
        self.yolo_publisher.unadvertise()
        self.target_info_publisher.unadvertise()
        self.client.terminate()
        print("Disconnected")


if __name__ == "__main__":
    model_path = os.environ.get("YOLO_MODEL_PATH", DEFAULT_MODEL)
    if not os.path.isfile(model_path):
        raise FileNotFoundError(f"YOLO model not found: {model_path}")

    ros_host = os.environ.get("ROS_HOST", "localhost")
    ros_port = int(os.environ.get("ROS_PORT", 9090))
    conf = float(os.environ.get("YOLO_CONF", "0.5"))

    yolo_processor = YOLOProcessor(model_path, ros_host, ros_port, conf)
    try:
        yolo_processor.start_processing()
        while yolo_processor.client.is_connected:
            pass
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        yolo_processor.stop_processing()
