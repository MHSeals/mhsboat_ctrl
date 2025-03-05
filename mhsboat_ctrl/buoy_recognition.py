import cv2
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import rclpy
from rclpy.node import Node
import os
import time
import urllib.request
from collections import defaultdict, deque
from typing import Dict
from sensor_msgs.msg import Image
from boat_interfaces.msg import AiOutput
from mhsboat_ctrl.utils.image_tools import preprocess
from mhsboat_ctrl.utils import rgb

print("Importing libraries...")
print("Libraries imported")
print("Checking for model...")

MODEL_URL = "https://github.com/MHSeals/buoy-model/releases/download/V14/best.pt"
file_name = os.path.basename(MODEL_URL)
model_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), file_name)

if not os.path.exists(model_path):
    print(f"Model not found; downloading {MODEL_URL}")
    urllib.request.urlretrieve(MODEL_URL, model_path)

print("Loading model...")
model = YOLO(model_path)
print("Model loaded")

# Constants
DISPLAY_RESOLUTION = (1280, 720)
MODEL_INPUT_DIMENSIONS = (1080, 1080)
DISPLAY_WIDTH, DISPLAY_HEIGHT = DISPLAY_RESOLUTION
MODEL_WIDTH, MODEL_HEIGHT = MODEL_INPUT_DIMENSIONS
X_SCALE_FACTOR = DISPLAY_WIDTH / MODEL_WIDTH
Y_SCALE_FACTOR = DISPLAY_HEIGHT / MODEL_HEIGHT
FRAME_AREA = MODEL_WIDTH * MODEL_HEIGHT

# Pre-made overlay for on-screen display
print("Creating overlay...")
overlay = np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)
cv2.putText(
    overlay, "Press k to pause", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
)
cv2.putText(
    overlay,
    "Press ESC to exit",
    (10, 75),
    cv2.FONT_HERSHEY_SIMPLEX,
    0.5,
    (0, 255, 0),
    1,
)
cv2.putText(
    overlay,
    "Press r to restart (video cap only)",
    (10, 100),
    cv2.FONT_HERSHEY_SIMPLEX,
    0.5,
    (0, 255, 0),
    1,
)
print("Overlay created")

colors: Dict[str, rgb] = {
    "blue_buoy": rgb(33, 49, 255),
    "dock": rgb(132, 66, 0),
    "green_buoy": rgb(135, 255, 0),
    "green_pole_buoy": rgb(0, 255, 163),
    "misc_buoy": rgb(255, 0, 161),
    "red_buoy": rgb(255, 0, 0),
    "red_pole_buoy": rgb(255, 92, 0),
    "yellow_buoy": rgb(255, 255, 0),
    "black_buoy": rgb(0, 0, 0),
    "red_racquet_ball": rgb(255, 153, 0),
    "yellow_racquet_ball": rgb(204, 255, 0),
    "blue_racquet_ball": rgb(102, 20, 219),
}


class CameraSubscriber(Node):
    def __init__(self):
        print("Initializing node")
        super().__init__("camera_subscriber")
        self.start_time = time.perf_counter()
        self.display_time = 1.0
        self.fc = 0
        self.FPS = 0
        self.total_frames = 0
        self.processing_times = []
        self.last_callback_time = time.perf_counter()

        self.cvbridge = CvBridge()
        # Use deque with fixed maxlen for efficient tracking history updates
        self.track_history = defaultdict(lambda: deque(maxlen=30))

        self.create_subscription(Image, "/color/image_raw", self.safe_image_callback, 10)
        self.publisher = self.create_publisher(AiOutput, "AiOutput", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.output = AiOutput()
        print("Node initialized")

    def timer_callback(self):
        if time.perf_counter() - self.last_callback_time > 1:
            print("No frames received in the last second")
        self.publisher.publish(self.output)
    
    def safe_image_callback(self, data: Image):
        try:
            self.image_callback(data)
        except Exception as e:
            print(f"Error in callback: {e}")

    def image_callback(self, data: Image):
        self.get_logger().info("Processing new frame")
        self.get_logger().info(f"Frame rate: {self.FPS}")
        self.last_callback_time = time.perf_counter()
        frame_start_time = time.perf_counter()
        self.total_frames += 1

        # Convert ROS image to OpenCV image and resize for display
        raw_frame = self.cvbridge.imgmsg_to_cv2(data, "bgr8")
        display_frame = cv2.resize(raw_frame, DISPLAY_RESOLUTION)
        original_frame = display_frame.copy()

        # Prepare frame for model prediction
        model_frame = cv2.resize(display_frame, MODEL_INPUT_DIMENSIONS)
        model_frame = preprocess(model_frame)

        # Update FPS calculation
        elapsed_time = time.perf_counter() - self.start_time
        self.fc += 1
        if elapsed_time >= self.display_time:
            self.FPS = self.fc / elapsed_time
            self.fc = 0
            self.start_time = time.perf_counter()

        fps_disp = f"FPS: {self.FPS:.2f}"
        cv2.putText(
            original_frame,
            fps_disp,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )
        original_frame = cv2.add(original_frame, overlay)

        # Initialize output message fields for this frame
        output = AiOutput()
        output.num = 0
        output.img_width = DISPLAY_WIDTH
        output.img_height = DISPLAY_HEIGHT
        output.types = []
        output.confidences = []
        output.lefts = []
        output.tops = []
        output.widths = []
        output.heights = []

        # Run model tracking
        results = model.track(model_frame, persist=True, tracker="bytetrack.yaml")

        # Create one annotator instance per frame
        annotator = Annotator(original_frame, line_width=1)

        for pred in results:
            if pred.boxes is None:
                continue

            names = pred.names
            for i in range(len(pred.boxes)):
                cls_id = int(pred.boxes.cls[i])
                name = names.get(cls_id, "Unknown")
                confidence = pred.boxes.conf[i]
                # Get and scale bounding box coordinates
                bbox = pred.boxes[i].xyxy[0]
                scaled_bbox = [
                    bbox[0] * X_SCALE_FACTOR,
                    bbox[1] * Y_SCALE_FACTOR,
                    bbox[2] * X_SCALE_FACTOR,
                    bbox[3] * Y_SCALE_FACTOR,
                ]
                x1, y1, x2, y2 = scaled_bbox

                # Disregard large bounding boxes
                area = (x2 - x1) * (y2 - y1)
                if area / FRAME_AREA > 0.50:
                    continue

                # Update output message
                output.num += 1
                output.types.append(name)
                output.confidences.append(int(confidence * 100))
                output.lefts.append(int(x1))
                output.tops.append(int(y1))
                output.widths.append(int(x2 - x1))
                output.heights.append(int(y2 - y1))

                center_x = x1 + (x2 - x1) / 2
                center_y = y1 + (y2 - y1) / 2

                detection_id = None
                color = colors.get(name, rgb(255, 255, 255))
                if pred.boxes.id is not None:
                    detection_id = int(pred.boxes.id[i])
                    # Update tracking history using deque (auto-discarding old points)
                    self.track_history[detection_id].append(
                        (int(center_x), int(center_y))
                    )
                    track_points = np.array(
                        self.track_history[detection_id], dtype=np.int32
                    )
                    if len(track_points) > 1:
                        original_frame = cv2.polylines(
                            original_frame,
                            [track_points.reshape((-1, 1, 2))],
                            isClosed=False,
                            color=color.as_bgr(),
                            thickness=2,
                        )

                # Optional debug print (consider removing or throttling in production)
                print(f"{name} {int(confidence * 100)}% {scaled_bbox}")

                label = f"{detection_id if detection_id is not None else 'None'}: {name} ({int(confidence * 100)}%) {int(area)}px"
                annotator.box_label(
                    (int(x1), int(y1), int(x2), int(y2)),
                    label,
                    color=color.as_bgr(),
                    txt_color=color.text_color().as_bgr(),
                )

        # Finalize annotation after processing all detections
        original_frame = annotator.result()

        frame_processing_time = time.perf_counter() - frame_start_time
        self.processing_times.append(frame_processing_time)
        self.processing_times = self.processing_times[-100:]
        print(
            f"Frame processing time: {frame_processing_time * 1000:.2f}ms, Average: {np.mean(self.processing_times) * 1000:.2f}ms"
        )

        if os.environ.get("DISPLAY", "") != "":
            cv2.imshow("result", original_frame)
            key = cv2.waitKey(1)
            if key == 27 or cv2.getWindowProperty("result", cv2.WND_PROP_VISIBLE) < 1:
                raise KeyboardInterrupt
        else:
            print("Headless mode: Skipping display")

        self.output = output  # Publish the latest detection output


def main(args=None):
    rclpy.init(args=args)
    cam_sub = CameraSubscriber()
    try:
        rclpy.spin(cam_sub)
    except KeyboardInterrupt:
        pass
    finally:
        cam_sub.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
