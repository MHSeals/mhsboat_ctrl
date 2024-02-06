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
from collections import defaultdict
import time
# from rgb import colors

model_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "v10.pt")
model = YOLO(model_path)
print("Model loaded")


# model.to("cuda")
class CameraSubscriber(Node):
    def __init__(self):
        super().__init__("camera_subscriber")
        self.create_subscription(Image, "/color/image_raw", self.callback, 10)
        #self.create_subscription(Image, "/wamv/sensors/cameras/front_left_camera_sensor/optical/image_raw", self.callback, 10)

        # create publisher that publishes bounding box coordinates and size and buoy type
        # int32 num -- um of buoys
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

    def timer_callback(self):
        print("publish")
        self.publisher.publish(self.output)

    def callback(self, data: Image):
        print("callback")

        class rgb:
            def __init__(self, r, g, b):
                self.r = r
                self.g = g
                self.b = b

            def __str__(self):
                return f"rgb({self.r}, {self.g}, {self.b})"

            def __repr__(self):
                return f"rgb({self.r}, {self.g}, {self.b})"

            def as_bgr(self) -> tuple:
                return (self.b, self.g, self.r)

            def invert(self):
                return rgb(255 - self.r, 255 - self.g, 255 - self.b)

            def text_color(self):
                luma = 0.2126 * self.r + 0.7152 * self.g + 0.0722 * self.b

                if luma < 40:
                    return rgb(255, 255, 255)
                else:
                    return rgb(0, 0, 0)

        colors: dict[str, rgb] = {
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
        track_history = defaultdict(lambda: [])

        # can change to use different webcams
        # cap = cv2.VideoCapture("PXL_20240108_222954915.TS.mp4")

        # if not cap.isOpened():
        # raise IOError("Cannot open webcam")

        start_time = time.perf_counter()

        display_time = 1
        fc = 0
        FPS = 0
        total_frames = 0
        prog_start = time.perf_counter()

        FRAME_SIZE = (1280, 720)

        IN_SIZE = (1280, 1280)

        self.camera_output = CvBridge().imgmsg_to_cv2(data, "bgr8")
        frame = self.camera_output
        # print("frame"+str(frame))
        frame = cv2.resize(frame, FRAME_SIZE)
        frame_copy = frame
        x_scale_factor = frame.shape[1] / IN_SIZE[0]
        y_scale_factor = frame.shape[0] / IN_SIZE[1]
        x_orig, y_orig = frame.shape[1], frame.shape[0]

        total_frames += 1
        TIME = time.perf_counter() - start_time

        frame = cv2.resize(frame, FRAME_SIZE)

        original_frame = frame.copy()
        frame = cv2.resize(frame, IN_SIZE)
        # print("frame size?: "+str(cv2.size(frame)))

        frame_area = frame.shape[0] * frame.shape[1]    

        fc += 1

        if (TIME) >= display_time:
            FPS = fc / (TIME)
            fc = 0
            start_time = time.perf_counter()

        fps_disp = "FPS: " + str(FPS)[:5]

        results = model.track(frame, persist=True, tracker="bytetrack.yaml")

        original_frame = cv2.putText(
            original_frame,
            fps_disp,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )

        original_frame = cv2.putText(
            original_frame,
            "Press k to pause",
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )

        original_frame = cv2.putText(
            original_frame,
            "Press ESC to exit",
            (10, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )

        original_frame = cv2.putText(
            original_frame,
            "Press r to restart (video cap only)",
            (10, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )

        num = 0
        types = []
        confidences = []
        tops = []
        lefts = []
        widths = []
        heights = []
        ids = []  # id of each buoy? will need to investigate with new model
        for pred in results:
            names = pred.names

            for i in range(len(pred.boxes)):
                num += 1
                name = names.get(int(pred.boxes.cls[i]))
                types.append(name)
                confidence = pred.boxes.conf[i]
                print(confidence)
                confidences.append(int(confidence * 100))

                bounding_box = pred.boxes[i].xyxy[0]
                bounding_box = [
                    bounding_box[0] * x_scale_factor,
                    bounding_box[1] * y_scale_factor,
                    bounding_box[2] * x_scale_factor,
                    bounding_box[3] * y_scale_factor,
                ]

                # Calculate area of bounding box

                area = (bounding_box[2] - bounding_box[0]) * (
                    bounding_box[3] - bounding_box[1]
                )

                # Disregard large bounding boxes

                if area / frame_area > 0.20:
                    continue

                tops.append(int(bounding_box[1]))
                lefts.append(int(bounding_box[0]))
                widths.append(int(bounding_box[2] - bounding_box[0]))
                heights.append(int(bounding_box[3] - bounding_box[1]))

                x, y = bounding_box[:2]
                w, h = bounding_box[2] - x, bounding_box[3] - y

                center_x = x + w / 2
                center_y = y + h / 2

                id = None

                color = colors.get(name, rgb(255, 255, 255))

                if pred.boxes.id is not None:
                    id = int(pred.boxes.id[i])
                    print("id: " + str(id))
                    track = track_history[id]
                    print("track: " + str(track))
                    track.append((float(center_x), float(center_y)))
                    if len(track) > 30:
                        track.pop(0)

                    ids.append(id)

                    # Draw the tracking lines
                    points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
                    original_frame = cv2.polylines(
                        original_frame,
                        [points],
                        isClosed=False,
                        color=color.as_bgr(),
                        thickness=2,
                    )

                print(f"{name} {int(confidence*100)}% {bounding_box}")

                # original_frame = cv2.putText(original_frame,
                #                              f"{id if id is not None else 'None'}: {name} ({int(confidence*100)})% {int(area)}px",
                #                              (int(bounding_box[0]), int(bounding_box[1])-5),
                #                              cv2.FONT_HERSHEY_SIMPLEX, 0.4, color.as_bgr(), 1)
                # original_frame = cv2.rectangle(original_frame,
                #                                (int(bounding_box[0]), int(bounding_box[1])),
                #                                (int(bounding_box[2]), int(bounding_box[3])),
                #                                color.as_bgr(), 1)

                annotator = Annotator(original_frame, line_width=1)

                annotator.box_label(
                    (x, y, x + w, y + h),
                    f"{id if id is not None else 'None'}: {name} ({int(confidence*100)})% {int(area)}px",
                    color=color.as_bgr(),
                    txt_color=color.text_color().as_bgr(),
                )

                original_frame = annotator.result()

        # bounding_box[0] = left side
        # bounding_box[1] = top
        # bounding_box[2] = right side
        # bounding_box[3] = bottom
        # print("types: "+str(types))
        # print("num: "+str(num))

        # sort all data to be in increasing id order
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                if ids[i] > ids[j]:
                    tempId = ids[i]
                    tempType = types[i]
                    tempConfidences = confidences[i]
                    tempLefts = lefts[i]
                    tempTops = tops[i]
                    tempWidths = widths[i]
                    tempHeights = heights[i]
                    ids[i] = ids[j]
                    types[i] = types[j]
                    confidences[i] = confidences[j]
                    lefts[i] = lefts[j]
                    tops[i] = tops[j]
                    widths[i] = widths[j]
                    heights[i] = heights[j]
                    ids[j] = tempId
                    types[j] = tempType
                    confidences[j] = tempConfidences
                    lefts[j] = tempLefts
                    tops[j] = tempTops
                    widths[j] = tempWidths
                    heights[j] = tempHeights

        self.output = AiOutput(
            num=num,
            img_width=IN_SIZE[0],
            img_height=IN_SIZE[1],
            types=types,
            confidences=confidences,
            lefts=lefts,
            tops=tops,
            widths=widths,
            heights=heights,
            ids=ids,
        )

        #commented out for xavier and jetson so no error when no screen
        #cv2.imshow("result", original_frame)
        t = time.time()

        print("t: "+str(t))
        if(int(t)%10==0):
            imgpath = path.join('/root/roboboat_ws/src/mhsboat_ctrl/logs/images/',str(t))
            cv2.imwrite(imgpath+"_raw.jpg",frame_copy)
            cv2.imwrite(imgpath+"_ai.jpg",frame)
            print("saved")

        c = cv2.waitKey(1)
        """
        if c == 107:
            time.sleep(0.1)
            while True:
                c = cv2.waitKey(1)
                if c == 107 or c == 27:
                    break

                if cv2.getWindowProperty("result", cv2.WND_PROP_VISIBLE) < 1:
                    break

                if c == 114:
                    track_history.clear()
                    break

        if c == 114:
            track_history.clear()

        cv2.destroyAllWindows()
        """
        print(f"Avg FPS: {total_frames / (time.perf_counter() - prog_start)}")
        """
        self.camera_output = CvBridge().imgmsg_to_cv2(data, "bgr8")

        # Hack to get color picker inside vscode
        class rgb():
            def __init__(self, r, g, b):
                self.r = r
                self.g = g
                self.b = b
            
            def __str__(self):
                return f"rgb({self.r}, {self.g}, {self.b})"

            def __repr__(self):
                return f"rgb({self.r}, {self.g}, {self.b})"
            
            def as_bgr(self) -> tuple:
                return (self.b, self.g, self.r)

        colors: dict[rgb] = {
            "blue_ball": rgb(0, 0, 255),
            "dock": rgb(109, 67, 3),
            "green_ball": rgb(0, 255, 0),
            "green_pole": rgb(0, 255, 0),
            "misc_buoy": rgb(0, 217, 255),
            "red_ball": rgb(255, 0, 0),
            "red_pole": rgb(255, 0, 0),
            "yellow_ball": rgb(255, 255, 0),
        }

        frame = self.camera_output
        original_frame = self.camera_output
        x_scale_factor = original_frame.shape[1] / 640
        y_scale_factor = original_frame.shape[0] / 640
        x_orig, y_orig = original_frame.shape[1], original_frame.shape[0]
        frame = cv2.resize(frame, (640, 640))

        result = model(frame)

        num = 0
        types = []
        confidences = []
        tops = []
        lefts = []
        widths = []
        heights = []
        for pred in result:
            names = pred.names
            for i in range(len(pred.boxes)):
                num += 1
                name = names.get(int(pred.boxes.cls[i]))
                types.append(name)
                confidence = pred.boxes.conf[i]
                print(confidence)
                confidences.append(int(confidence*100))
                bounding_box = pred.boxes[i].xyxy[0]
                bounding_box = [
                    bounding_box[0] * x_scale_factor,
                    bounding_box[1] * y_scale_factor,
                    bounding_box[2] * x_scale_factor,
                    bounding_box[3] * y_scale_factor
                ]

                tops.append(int(bounding_box[1]))
                lefts.append(int(bounding_box[0]))
                widths.append(int(bounding_box[2]-bounding_box[0]))
                heights.append(int(bounding_box[3]-bounding_box[1]))

                print(f"{name} {int(confidence*100)}% {bounding_box}")

                color = colors.get(name, rgb(255, 255, 255))

                original_frame = cv2.putText(original_frame, 
                                            f"{name} {int(confidence*100)}%",
                                            (int(bounding_box[0]), int(bounding_box[1])-5),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color.as_bgr(), 1)
                original_frame = cv2.rectangle(original_frame,
                                            (int(bounding_box[0]), int(bounding_box[1])),
                                            (int(bounding_box[2]), int(bounding_box[3])), 
                                            color.as_bgr(), 1)

        # original_frame = cv2.putText(original_frame, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        #bounding_box[0] = left side
        #bounding_box[1] = top
        #bounding_box[2] = right side
        #bounding_box[3] = bottom
        print(types)
        self.output = AiOutput(num=num,img_width=data.width,img_height=data.height,types=types,confidences=confidences,lefts=lefts,tops=tops,widths=widths,heights=heights)
        cv2.imshow("result", original_frame)
        cv2.waitKey(1)
        # if c == 27:
        #     break

        # if cv2.getWindowProperty("result", cv2.WND_PROP_VISIBLE) < 1:
        #     break
        """


def main(args=None):
    rclpy.init(args=args)
    cam_sub = CameraSubscriber()
    rclpy.spin(cam_sub)
    cam_sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
