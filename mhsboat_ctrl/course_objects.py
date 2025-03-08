from typing import Literal, TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from mhsboat_ctrl.utils.custom_types import numeric

from mhsboat_ctrl.enums import BuoyColors, Shapes


class CourseObject:
    def __init__(self, x: "numeric", y: "numeric", size: "numeric"):
        self.x = x
        self.y = y
        self.size = size

    def __str__(self):
        return f"CourseObject({self.x}, {self.y})"


class Shape(CourseObject):
    def __init__(
        self, x: "numeric", y: "numeric", shape: Shapes, color: BuoyColors, size: "numeric"
    ):
        self.x = x
        self.y = y
        self.shape = shape
        self.color = color
        self.size = size

    def __str__(self):
        return f"Shape({self.x}, {self.y}, {self.shape}, {self.color})"


class Buoy(CourseObject):
    def __init__(self, x: "numeric", y: "numeric", color: BuoyColors, size: "numeric"):
        self.x = x
        self.y = y
        self.color = color
        self.size = size

    def __str__(self):
        return f"Buoy({self.x}, {self.y})"


class PoleBuoy(Buoy):
    def __init__(
        self,
        x: "numeric",
        y: "numeric",
        color: Literal[BuoyColors.RED, BuoyColors.GREEN],
        size: "numeric"
    ):
        self.x = x
        self.y = y
        self.color = color
        self.size = size

    def __str__(self):
        return f"PoleBuoy({self.x}, {self.y}, {self.color})"


class BallBuoy(Buoy):
    def __init__(self, x: "numeric", y: "numeric", z: "numeric", color: BuoyColors, size: "numeric"):
        self.x = x
        self.y = y
        self.color = color
        self.size = size

    def __str__(self):
        return f"BallBuoy({self.x}, {self.y}, {self.color})"