from task import Task
from mhsboat_ctrl.enums import TaskCompletionStatus, TaskStatus

class TaskOne(Task):
    status = TaskStatus.NOT_STARTED

    def search(self) -> None | tuple[int, int]:
        print("Searching for TaskOne")
        return (0, 0)
    
    def run(self) -> TaskCompletionStatus:
        print("Running TaskOne")
        return TaskCompletionStatus.SUCCESS