import abc
from typing import ClassVar

from enums import TaskCompletionStatus, TaskStatus


class Task(abc.ABC):
    status: ClassVar[TaskStatus]

    def __init_subclass__(cls, **kwargs):
        for var in ['status']:
            if not hasattr(cls, var):
                raise NotImplementedError(
                    f'Class {cls} lacks required `{var}` class attribute'
                )

    @abc.abstractmethod
    def search(self) -> None | tuple[int, int]:
        ...

    @abc.abstractmethod
    def run(self) -> TaskCompletionStatus:
        ...