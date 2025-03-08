"""
Currently, I am using the size of the bounding boxes to approximate their closeness
and determine which ones should be used as pairs for the boat's path but if necessary,
we may subscribe to the depth stream of the stereo camera and take the average pixel
depth within the bounding box to get which ones are the closes. This approach is likely
overkill though because we don't use the depth values for anything else.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSLivelinessPolicy,
)

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu, CameraInfo
import tf_transformations
from boat_interfaces.msg import BuoyDetections

from mhsboat_ctrl.enums import TaskStatus, BuoyColors
from mhsboat_ctrl.task import Task
from mhsboat_ctrl.course_objects import Shape, PoleBuoy, BallBuoy
from mhsboat_ctrl.pid import PIDController

import os
import glob
import importlib
from typing import List

KP = 1
KI = 0.05
KD = 0.1
INTEGRAL_BOUND = 1

buoy_color_mapping: Dict[str, BuoyColors] = {
    "black": BuoyColors.BLACK,
    "blue": BuoyColors.BLUE,
    "green": BuoyColors.GREEN,
    "red": BuoyColors.RED,
    "yellow": BuoyColors.YELLOW,
}

class VisionBoatController(Node):
    tasks: List[Task] = []

    def __init__(self):
        super().__init__("vision_mhsboat_ctrl")
        
        self.get_logger().info("Boat Controller Node Initialized")
        
        self.buoy_detection_subscriber = self.create_subscription(
            BuoyDetections, "buoy_detections", self.buoy_detection_callback, 10    
        )

        self.intrinsics_subscriber = self.create_subscription(
            CameraInfo, '/depth/camera_info', self.intrinsics_callback, 10
        )
        self.intrinsics = None


        self.camera_info_subscriber = self.create_subscription(
            CameraInfo, '/color/camera_info', self.camera_info_callback, 10
        )
        self.width = None
        self.height = None

        self.qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )
        
        self.imu_sub = self.create_subscription(
            Imu, "/mavros/imu/data", self.imu_callback, self.qos_profile
        )

        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10
        )

        self.cmd_vel = TwistStamped()
        self.cmd_vel.twist.linear.x = 0.0
        self.cmd_vel.twist.linear.y = 0.0
        self.cmd_vel.twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.cmd_vel)

        self.buoys = []

        self.pid = PIDController(self, KP, KI, KD, INTEGRAL_BOUND)
        
        self.orientation = 0

        for task_file in glob.glob(os.path.join(os.path.dirname(__file__), "tasks", "*.py")):
            if os.path.basename(task_file) == "__init__.py":
                continue

            task_module = importlib.import_module(f"mhsboat_ctrl.tasks.{os.path.basename(task_file)[:-3]}")
            task_module.main(self)

        self.run()

    def camera_info_callback(self, msg: CameraInfo):
        self.width = msg.width
        self.height = msg.height
        self.get_logger().info(f"Image Width: {self.width}, Height: {self.height}")

    def intrinsics_callback(self, msg) -> None:
        """ Get camera intrinsics from CameraInfo topic. """
        fx = msg.k[0]  # Focal length in x
        fy = msg.k[4]  # Focal length in y
        cx = msg.k[2]  # Optical center x
        cy = msg.k[5]  # Optical center y
        
        self.intrinsics = (fx, fy, cx, cy)
        
    def imu_callback(self, msg: Imu) -> None:
        self.orientation += msg.angular_velocity.z
        
    def buoy_detection_callback(self, camera_data: BuoyDetections) -> None:
        lefts = np.array([camera_data.lefts])
        tops = np.array([camera_data.tops])
        widths = np.array([camera_data.widths])
        heights = np.array([camera_data.heights])
        self.buoys = []
        
        centers = np.array([lefts + widths / 2, tops + heights / 2]).T

        for i in range(msg.num):
            x = centers[i, 0]
            y = centers[i, 1]
            size = widths[i] * heights[i]

            if camera_data.types[i].endswith("pole_buoy"):
                buoy_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                # type: ignore - this will always be either red or green
                self.buoys.append(PoleBuoy(x, y, buoy_color, size)) # type: ignore
            elif camera_data.types[i].endswith("buoy"):
                buoy_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                self.buoys.append(BallBuoy(x, y, buoy_color, size))
            elif camera_data.types[i].endswith("circle"):
                shape_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                self.buoys.append(Shape(x, y, Shapes.CIRCLE, shape_color, size))
            elif camera_data.types[i].endswith("triangle"):
                shape_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                self.buoys.append(Shape(x, y, Shapes.TRIANGLE, shape_color, size))
            elif camera_data.types[i].endswith("cross"):
                shape_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                self.buoys.append(Shape(x, y, Shapes.CROSS, shape_color, size))
            elif camera_data.types[i].endswith("square"):
                shape_color = buoy_color_mapping[camera_data.types[i].split("_")[0]]
                self.buoys.append(Shape(x, y, Shapes.SQUARE, shape_color, size))
            elif camera_data.types[i].endswith("duck_image"):
                pass
            elif camera_data.types[i].endswith("racquet_ball"):
                pass

    def set_forward_velocity(self, velocity: float):
        self.cmd_vel.twist.linear.x = velocity
        self.cmd_vel_publisher.publish(self.cmd_vel)

    def set_backward_velocity(self, velocity: float):
        # mavros is just really weird
        self.cmd_vel.twist.linear.y = velocity
        self.cmd_vel_publisher.publish(self.cmd_vel)

    def set_angular_velocity(self, velocity: float):
        self.cmd_vel.twist.angular.z = velocity
        self.cmd_vel_publisher.publish(self.cmd_vel)

    def add_task(self, task: Task):
        """
        Add a task to the controller

        :param task: The task to add
        :type task: class:`mhsboat_ctrl.task.Task`
        """
        self.tasks.append(task)
    
    def _process_tasks(self):
        """
        Run all tasks
        """

        for task in self.tasks:
            if task.status == TaskStatus.COMPLETED:
                continue

            search_result = task.search()
            if search_result is not None:
                self.get_logger().info(f"Found Task {task.__class__.__qualname__}")

                try:
                    completion_status = task.run()
                except KeyboardInterrupt:
                    self.get_logger().info("Task cancelled")
                    completion_status = TaskStatus.FAILURE
                except Exception as e:
                    self.get_logger().error(f"Task failed: {e}")
                    completion_status = TaskStatus.FAILURE
                
                if completion_status == TaskStatus.SUCCESS:
                    self.get_logger().info("Task completed")
                    self.set_forward_velocity(0)
                    self.set_angular_velocity(0)
                elif completion_status == TaskStatus.FAILURE:
                    self.get_logger().error("Task failed/cancelled")
                    self.set_forward_velocity(0)
                    self.set_angular_velocity(0)

    def run(self):
        """
        Run the boat controller
        """
        self.get_logger().info("Running Boat Controller")
        while rclpy.ok():
            try:
                self._process_tasks()
            except KeyboardInterrupt:
                exit = input("Exit? (y/n): ")
                if exit == "y":
                    break
                else:
                    continue
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                continue


def main(args=None):
    rclpy.init(args=args)

    boat_controller = VisionBoatController()

    try:
        rclpy.spin(boat_controller)
    except KeyboardInterrupt:
        boat_controller.get_logger().info("Shutting down")
    finally:
        boat_controller.destroy_node()
        rclpy.shutdown()