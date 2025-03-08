from enum import Enum
class TaskStatus(Enum):
    SUCCESS = 0
    FAILURE = 1
    ACTIVE = 2
    SEARCHING = 3

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
