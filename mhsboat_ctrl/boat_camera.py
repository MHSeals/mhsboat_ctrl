import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import json, sys

model = YOLO("/root/ros_ws/src/mhsboat_ctrl/mhsboat_ctrl/v9.pt")
print("Model loaded")


class CameraService(Node):
    def __init__(self):
        super().__init__("camera_service")
        try:
            self.show = eval(sys.argv[1])
        except IndexError:
            self.show = False

        self.create_subscription(
            Image,
            "/color/image_raw",
            self.callback,
            10,
        )
        # /wamv/sensors/cameras/front_left_camera_sensor/optical/image_raw

        self.camera_publisher = self.create_publisher(String, "/taskone/poles", 10)

    def callback(self, data: Image):
        # Convert camera topic output to cv2 processable data
        self.camera_output = CvBridge().imgmsg_to_cv2(data, "bgr8")

        # Hack to get color picker inside vscode
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

        # Since model is trained to recognize buoys on a 640x640 image,
        # scale camera output down to that size, then scale back up so we can view it normally
        frame = self.camera_output
        original_frame = self.camera_output
        x_scale_factor = original_frame.shape[1] / 640
        y_scale_factor = original_frame.shape[0] / 640
        x_orig, y_orig = original_frame.shape[1], original_frame.shape[0]
        frame = cv2.resize(frame, (640, 640))
        result = model(frame)

        poles = []
        for pred in result:
            names = pred.names

            for i in range(len(pred.boxes)):
                name = names.get(int(pred.boxes.cls[i]))
                confidence = pred.boxes.conf[i]
                bounding_box = pred.boxes[i].xyxy[0]
                bounding_box = [
                    bounding_box[0] * x_scale_factor,
                    bounding_box[1] * y_scale_factor,
                    bounding_box[2] * x_scale_factor,
                    bounding_box[3] * y_scale_factor,
                ]

                # Add all poles for task one
                if "pole" in name:
                    x_left = int(bounding_box[0])
                    y_top = int(bounding_box[1])
                    x_right = int(bounding_box[2])
                    y_bottom = int(bounding_box[3])
                    length_x = int(bounding_box[2]) - int(bounding_box[0])
                    length_y = int(bounding_box[3]) - int(bounding_box[1])
                    area = length_x * length_y
                    # Filtering out weird pole data on bottom of screen
                    if y_top < 535:
                        pole_data = {
                            "name": name,
                            "area": area,
                            "x_left": x_left,
                            "x_right": x_right,
                            "y_top": y_top,
                            "y_bottom": y_bottom,
                        }
                        poles.append(pole_data)

                # print(f"{name} {int(confidence*100)}% {bounding_box}")

                # Draw boxes and text onto screen
                color = colors.get(name, rgb(255, 255, 255))
                original_frame = cv2.putText(
                    original_frame,
                    f"{name} {int(confidence*100)}%",
                    (int(bounding_box[0]), int(bounding_box[1]) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    color.as_bgr(),
                    1,
                )
                original_frame = cv2.rectangle(
                    original_frame,
                    (int(bounding_box[0]), int(bounding_box[1])),
                    (int(bounding_box[2]), int(bounding_box[3])),
                    color.as_bgr(),
                    1,
                )

        pole_data = String()
        pole_data.data = json.dumps(poles)
        self.camera_publisher.publish(pole_data)

        if self.show:
            cv2.imshow("result", original_frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    camera = CameraService()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


"""
buoy_middle = (left_buoy["x_right"] - right_buoy["x_left"]) / 2 + SCREEN_SIZE
"""
