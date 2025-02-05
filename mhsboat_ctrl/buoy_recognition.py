#!/usr/bin/env python3.10

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
from enum import Enum
from boat_interfaces.msg import AiOutput
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import os
from os import path
import time
import numpy as np
import numpy.typing as npt
from collections import defaultdict
import time
import urllib.request
from typing import Dict

from mhsboat_ctrl.utils import rgb
from mhsboat_ctrl.utils.image_tools import preprocess

MODEL_URL = "https://github.com/MHSeals/buoy-model/releases/download/V14/best.pt"

file_name = path.basename(MODEL_URL)

model_path = os.path.join(os.path.dirname(
    os.path.realpath(__file__)), file_name)

if not os.path.exists(model_path):
    print(f"Model not found; downloading {MODEL_URL}")
    urllib.request.urlretrieve(
        MODEL_URL, model_path)

model = YOLO(model_path)
print("Model loaded")

DISP_SIZE = (1280, 720)
IN_SIZE = (1080, 1080)

print("Creating overlay...")
overlay = np.zeros_like(np.zeros(DISP_SIZE, dtype=np.uint8))

overlay = cv2.putText(overlay, "Press k to pause",
                      (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

overlay = cv2.putText(overlay, "Press ESC to exit",
                      (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

overlay = cv2.putText(overlay, "Press r to restart (video cap only)", (
    10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

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
        self.display_time = 1
        self.fc = 0
        self.FPS = 0
        self.total_frames = 0
        self.prog_start = time.perf_counter()

        self.cvbridge = CvBridge()

        self.track_history = defaultdict(lambda: [])

        self.create_subscription(Image, "/color/image_raw", self.callback, 10)
        # self.create_subscription(Image, "/wamv/sensors/cameras/front_left_camera_sensor/optical/image_raw", self.callback, 10)

        # create publisher that publishes bounding box coordinates and size and buoy type
        # int32 num -- num of buoys
        # int32 img_width -- width of image
        # int32 img_height -- height of image
        # string[] types -- type of buoys
        # int32[] confidences -- confidence of being buoy
        # int32[] lefts -- top of bounding box coordinate
        # int32[] tops -- left of bounding box coordinate
        # int32[] widths -- widths of bounding boxes
        # int32[] heights -- heights of bounding boxes

        self.publisher = self.create_publisher(AiOutput, "AiOutput", 10)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.output = AiOutput()

        print("Node initialized")

    def timer_callback(self):
        self.publisher.publish(self.output)

    def callback(self, data: Image):
        x_scale_factor = data.width / DISP_SIZE[0]
        y_scale_factor = data.height / DISP_SIZE[1]
        x_original, y_original = data.width, data.height

        frame_start_time = time.perf_counter()
        self.total_frames += 1
        TIME = time.perf_counter() - self.start_time

        frame = self.cvbridge.imgmsg_to_cv2(data, "bgr8")

        frame = cv2.resize(src=frame, dsize=DISP_SIZE)

        frame = preprocess(frame)

        original_frame = frame.copy()

        frame = cv2.resize(frame, IN_SIZE)

        frame_area = frame.shape[0] * frame.shape[1]

        self.fc += 1

        if (TIME) >= self.display_time:
            self.FPS = self.fc / (TIME)
            self.fc = 0
            self.start_time = time.perf_counter()

        fps_disp = "FPS: " + str(self.FPS)[:5]

        frame = preprocess(frame)

        results = model.track(frame, persist=True, tracker="bytetrack.yaml")

        original_frame = cv2.putText(
            original_frame, fps_disp, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        original_frame = cv2.addWeighted(original_frame, 1, overlay, 0.5, 0)

        self.output.num = len(results)
        self.output.img_width = x_original
        self.output.img_height = y_original
        self.output.types = []
        self.output.confidences = []
        self.output.lefts = []
        self.output.tops = []
        self.output.widths = []
        self.output.heights = []

        for pred in results:
            names = pred.names

            # TODO: sometimes, on a frame with lots of objects, the model will only detect 1 object
            # for some reason, it is always on the same frames

            if pred.boxes is not None:
                for i in range(len(pred.boxes)):
                    name = names.get(int(pred.boxes.cls[i]))
                    confidence = pred.boxes.conf[i]
                    bounding_box = pred.boxes[i].xyxy[0]
                    bounding_box = [
                        bounding_box[0] * x_scale_factor,
                        bounding_box[1] * y_scale_factor,
                        bounding_box[2] * x_scale_factor,
                        bounding_box[3] * y_scale_factor
                    ]

                    # Calculate area of bounding box

                    area = (bounding_box[2] - bounding_box[0]) * \
                        (bounding_box[3] - bounding_box[1])

                    # Disregard large bounding boxes

                    if area / frame_area > 0.20:
                        continue

                    x, y = bounding_box[:2]
                    w, h = bounding_box[2] - x, bounding_box[3] - y

                    self.output.types.append(name)
                    self.output.confidences.append(int(confidence*100))
                    self.output.lefts.append(int(x))
                    self.output.tops.append(int(y))
                    self.output.widths.append(int(w))
                    self.output.heights.append(int(h))

                    center_x = x + w / 2
                    center_y = y + h / 2

                    id = None

                    color = colors.get(name, rgb(255, 255, 255))

                    if pred.boxes.id is not None:
                        id = int(pred.boxes.id[i])

                        track = self.track_history[id]
                        track.append((float(center_x), float(center_y)))
                        if len(track) > 30:
                            track.pop(0)

                        # Draw the tracking lines
                        points = np.hstack(track).astype(
                            np.int32).reshape((-1, 1, 2))
                        original_frame = cv2.polylines(
                            original_frame, [points], isClosed=False, color=color.as_bgr(), thickness=2)

                    print(f"{name} {int(confidence*100)}% {bounding_box}")

                    annotator = Annotator(original_frame, line_width=1)

                    annotator.box_label((x, y, x+w, y+h), f"{id if id is not None else 'None'}: {name} ({int(confidence*100)})% {int(area)}px",
                                        color=color.as_bgr(), txt_color=color.text_color().as_bgr())

                    original_frame = annotator.result()

        cv2.imshow("result", original_frame)
        c = cv2.waitKey(1)

        if c == 27:
            raise KeyboardInterrupt

        if cv2.getWindowProperty("result", cv2.WND_PROP_VISIBLE) < 1:
            raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    cam_sub = CameraSubscriber()
    rclpy.spin(cam_sub)
    cam_sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
