import rclpy
from rclpy.node import Node
import glob

from mhsboat_ctrl.enums import TaskCompletionStatus, TaskStatus
from mhsboat_ctrl.sensors import Sensors
from mhsboat_ctrl.task import Task

class BoatController(Node):
    tasks = []

    def __init__(self):
        super().__init__('mhsboat_ctrl')

        self.sensors = Sensors()

        for task_file in glob.glob("tasks/*.py"):
            task_module = __import__(task_file[:-3].replace("/", "."), fromlist=[""])
            task_module.main(self)

        self.get_logger().info("Boat Controller Node Initialized")

        self.run()

    def add_task(self, task: Task):
        self.tasks.append(task)

    def run(self):
        self.get_logger().info("Running tasks")
        while rclpy.ok():
            for task in self.tasks:
                if task.status == TaskStatus.COMPLETED:
                    continue

                search_result = task.search()
                if search_result is not None:
                    assert type(search_result) == tuple[int, int]

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

    rclpy.spin(boat_controller)

    boat_controller.destroy_node()
    rclpy.shutdown()
