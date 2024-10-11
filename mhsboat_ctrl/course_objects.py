from typing import Literal

from mhsboat_ctrl.enums import BuoyColors, Shapes

class CourseObject:
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y

    def __str__(self):
        return f"CourseObject({self.x}, {self.y})"


class Shape(CourseObject):
    def __init__(self, x: int, y: int, shape: Shapes, color: BuoyColors):
        self.x = x
        self.y = y
        self.shape = shape
        self.color = color

    def __str__(self):
        return f"Shape({self.x}, {self.y}, {self.shape}, {self.color})"


class Buoy(CourseObject):
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y

    def __str__(self):
        return f"Buoy({self.x}, {self.y})"


class PoleBuoy(Buoy):
    def __init__(self, x: int, y: int, color: Literal[BuoyColors.RED, BuoyColors.GREEN]):
        self.x = x
        self.y = y
        self.color = color

    def __str__(self):
        return f"PoleBuoy({self.x}, {self.y}, {self.color})"


class BallBuoy(Buoy):
    def __init__(self, x: int, y: int, color: BuoyColors):
        self.x = x
        self.y = y
        self.color = color

    def __str__(self):
        return f"BallBuoy({self.x}, {self.y}, {self.color})"