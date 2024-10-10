from mhsboat_ctrl.mhsboat_ctrl import BoatController
from mhsboat_ctrl.task import Task
from mhsboat_ctrl.enums import TaskCompletionStatus, TaskStatus


class TaskOne(Task):
    status = TaskStatus.NOT_STARTED

    def __init__(self, sensors):
        super().__init__(sensors)

    def search(self) -> None | tuple[int, int]:
        print("Searching for TaskOne")
        return (0, 0)

    def run(self) -> TaskCompletionStatus:
        print("Running TaskOne")
        return TaskCompletionStatus.SUCCESS


def main(controller: BoatController):
    controller.add_task(TaskOne(controller.sensors))
