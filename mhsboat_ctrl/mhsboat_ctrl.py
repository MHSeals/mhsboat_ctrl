import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import os
import glob
import importlib

from mhsboat_ctrl.enums import TaskCompletionStatus, TaskStatus
from mhsboat_ctrl.sensors import Sensors, SensorsSimulated
from mhsboat_ctrl.task import Task

class BoatController(Node):
    tasks: list[Task] = []

    def __init__(self):
        super().__init__('mhsboat_ctrl')

        self.declare_parameter("use_simulated_map", False)
        self.declare_parameter("map_file", "")

        use_simulated_map = self.get_parameter("use_simulated_map").get_parameter_value().bool_value
        map_file = self.get_parameter("map_file").get_parameter_value().string_value

        if use_simulated_map:
            if map_file == "":
                self.get_logger().error("Simulated map requested, but no map file specified")
                return
            
            self.get_logger().info("Using Simulated Map")
            self.sensors = SensorsSimulated(map_file)
        else:
            self.sensors = Sensors()

        for task_file in glob.glob(os.path.join(os.path.dirname(__file__), "tasks", "*.py")):
            if os.path.basename(task_file) == "__init__.py":
                continue

            task_module = importlib.import_module(f"mhsboat_ctrl.tasks.{os.path.basename(task_file)[:-3]}")
            task_module.main(self)

        self.get_logger().info("Boat Controller Node Initialized")

        self.run()

        self.sensors.destroy_node()

    def add_task(self, task: Task):
        self.tasks.append(task)

    def run(self):
        self.get_logger().info("Running Boat Controller")
        while rclpy.ok():
            for task in self.tasks:
                if task.status == TaskStatus.COMPLETED:
                    continue

                search_result = task.search()
                if search_result is not None:
                    self.get_logger().info(f"Found Task at {search_result[0]}, {search_result[1]}")

                    inp = ""
                    while True:
                        inp = input("Run task? (y/n): ")
                        if inp == "y" or inp == "n":
                            break
                        else:
                            print("Invalid input. Please enter 'y' or 'n'.")

                    if inp == "y":
                        completion_status = task.run()
                        if completion_status == TaskCompletionStatus.SUCCESS:
                            task.status = TaskStatus.COMPLETED
                        else:
                            self.get_logger().error("Task failed")

def main(args=None):
    rclpy.init(args=args)

    boat_controller = BoatController()

    try:
        rclpy.spin(boat_controller)
    except KeyboardInterrupt:
        boat_controller.get_logger().info("Shutting down")
    finally:
        boat_controller.sensors.destroy_node()
        boat_controller.destroy_node()
        rclpy.shutdown()
