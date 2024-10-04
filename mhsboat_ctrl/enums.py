from enum import Enum

class TaskCompletionStatus(Enum):
    SUCCESS = 0
    FAILURE = 1
    CANCELLED = 2
    PARTIAL_SUCCESS = 3

class TaskStatus(Enum):
    NOT_STARTED = 0
    FOUND = 1
    IN_PROGRESS = 2
    COMPLETED = 3