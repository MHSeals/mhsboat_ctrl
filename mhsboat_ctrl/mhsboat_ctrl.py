import rclpy
from rclpy.node import Node

from enums import TaskCompletionStatus, TaskStatus
from tasks.taskone import TaskOne

class BoatController(Node):
    tasks = []

    def __init__(self):
        super().__init__('mhsboat_ctrl')

        self.add_task(TaskOne())

        self.run()

    def add_task(self, task):
        self.tasks.append(task)

    def run(self):
        while rclpy.ok():
            for task in self.tasks:
                if task.status == TaskStatus.COMPLETED:
                    continue

                search_result = task.search()
                if search_result is not None:
                    assert type(search_result) == tuple[int, int]

                    print(f"Found Task at {search_result[0]}, {search_result[1]}")

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
                            print("Task failed to run.")

def main(args=None):
    rclpy.init(args=args)

    boat_controller = BoatController()

    rclpy.spin(boat_controller)

    boat_controller.destroy_node()
    rclpy.shutdown()
