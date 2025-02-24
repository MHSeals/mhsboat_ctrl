import abc
from typing import Tuple, Optional

from mhsboat_ctrl.enums import TaskCompletionStatus, TaskStatus
from mhsboat_ctrl.mhsboat_ctrl import BoatController


class Task(abc.ABC):
    status: TaskStatus

    def __init__(self, boat_controller: BoatController):
        self.boat_controller = boat_controller

    def __init_subclass__(cls, **kwargs):
        for var in ["status"]:
            if not hasattr(cls, var):
                raise NotImplementedError(
                    f"Class {cls} lacks required `{var}` class attribute"
                )

    @abc.abstractmethod
    def search(self) -> Optional[Tuple[float, float]]: ...

    @abc.abstractmethod
    def run(self) -> TaskCompletionStatus: ...
