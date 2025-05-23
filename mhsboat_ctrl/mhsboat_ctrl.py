import rclpy
from rclpy.node import Node
from typing import List
from geometry_msgs.msg import TwistStamped
from boat_interfaces.msg import BuoyMap
from uuid import UUID

from mhsboat_ctrl.enums import TaskCompletionStatus, TaskStatus, BuoyColors
from mhsboat_ctrl.task import Task
from mhsboat_ctrl.course_objects import CourseObject, PoleBuoy, BallBuoy

from mhsboat_ctrl.pid import PIDController

import os
import glob
import importlib

LOOKAHEAD = 3 # meters
KP = 1
KI = 0.05
KD = 0.1
INTEGRAL_BOUND = 1

class BoatController(Node):
    tasks: List[Task] = []

    def __init__(self):
        super().__init__("mhsboat_ctrl")

        # self.declare_parameter("map_file", "")

        self.pid = PIDController(self, LOOKAHEAD, KP, KI, KD, INTEGRAL_BOUND)
        self.buoy_map: list[CourseObject] = []

        for task_file in glob.glob(os.path.join(os.path.dirname(__file__), "tasks", "*.py")):
            if os.path.basename(task_file) == "__init__.py":
                continue

            task_module = importlib.import_module(f"mhsboat_ctrl.tasks.{os.path.basename(task_file)[:-3]}")
            task_module.main(self)

        self.sensors_subscriber = self.create_subscription(
            BuoyMap, "/mhsboat_ctrl/map", self.sensors_callback, 10
        )

        self.get_logger().info("Boat Controller Node Initialized")


        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10
        )

        self.cmd_vel = TwistStamped()
        self.cmd_vel.twist.linear.x = 0.0
        self.cmd_vel.twist.linear.y = 0.0
        self.cmd_vel.twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.cmd_vel)

        self.run()

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

    def sensors_callback(self, msg: BuoyMap):
        for i in range(len(msg.x)):
            color = BuoyColors(msg.colors[i].lower())
            if msg.types[i] == "pole":
                self.buoy_map.append(PoleBuoy(msg.x[i], msg.y[i], msg.z[i], color))  # type: ignore color will always only be red or green
            elif msg.types[i] == "ball":
                self.buoy_map.append(BallBuoy(msg.x[i], msg.y[i], msg.z[i], color))
            elif msg.types[i] == "shape":
                self.buoy_map.append(CourseObject(msg.x[i], msg.y[i], msg.z[i]))
            elif msg.types[i] == "course_object":
                self.buoy_map.append(CourseObject(msg.x[i], msg.y[i], msg.z[i]))
            else:
                self.get_logger().error(f"Unknown buoy type: {msg.types[i]}")
                continue

            self.buoy_map[-1].uid = UUID(hex=msg.uids[i])

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
                self.get_logger().info(f"Found Task {task.__qualname__}")

                inp = ""
                while True:
                    inp = input("Run task? (y/n): ")
                    if inp == "y" or inp == "n":
                        break
                    else:
                        print("Invalid input. Please enter 'y' or 'n'.")

                if inp == "y":
                    self.get_logger().info("Running task")
                    self.get_logger().info("Press Ctrl+C to cancel")

                    try:
                        completion_status = task.run()
                    except KeyboardInterrupt:
                        completion_status = TaskCompletionStatus.CANCELLED
                    except Exception as e:
                        self.get_logger().error(f"Task failed: {e}")
                        completion_status = TaskCompletionStatus.FAILURE
                    
                    if completion_status == TaskCompletionStatus.SUCCESS:
                        task.status = TaskStatus.COMPLETED
                        self.get_logger().info("Task completed")
                    elif completion_status == TaskCompletionStatus.PARTIAL_SUCCESS:
                        task.status = TaskStatus.PARTIAL_COMPLETION
                        self.get_logger().info("Task partially completed")
                    elif completion_status == TaskCompletionStatus.FAILURE:
                        task.status = TaskStatus.FAILURE
                        self.get_logger().error("Task failed")
                    elif completion_status == TaskCompletionStatus.CANCELLED:
                        self.get_logger().info("Task cancelled")
                    elif completion_status == TaskCompletionStatus.NOT_STARTED:
                        task.status = TaskStatus.NOT_STARTED
                        self.get_logger().error("Task not started")
                    else:
                        task.status = TaskStatus.FAILURE
                        self.get_logger().error("Task failed")

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

    boat_controller = BoatController()

    try:
        rclpy.spin(boat_controller)
    except KeyboardInterrupt:
        boat_controller.get_logger().info("Shutting down")
    finally:
        boat_controller.destroy_node()
        rclpy.shutdown()