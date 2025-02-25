from enum import Enum


class TaskCompletionStatus(Enum):
    SUCCESS = 0
    FAILURE = 1
    CANCELLED = 2
    PARTIAL_SUCCESS = 3
    NOT_STARTED = 4


class TaskStatus(Enum):
    NOT_STARTED = 0
    FOUND = 1
    IN_PROGRESS = 2
    COMPLETED = 3
    CANCELLED = 4
    FAILURE = 5
    PARTIAL_COMPLETION = 6


class BuoyColors(Enum):
    RED = "red"
    GREEN = "green"
    YELLOW = "yellow"
    BLUE = "blue"
    BLACK = "black"


class Shapes(Enum):
    CIRCLE = "circle"
    SQUARE = "square"
    TRIANGLE = "triangle"
    CROSS = "cross"
