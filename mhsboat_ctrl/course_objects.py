from typing import Literal, TYPE_CHECKING, Optional
from uuid import UUID

if TYPE_CHECKING:
    from mhsboat_ctrl.utils.custom_types import numeric

from mhsboat_ctrl.enums import BuoyColors, Shapes


class CourseObject:
    def __init__(self, x: "numeric", y: "numeric", z: "numeric"):
        self.x = x
        self.y = y
        self.z = z
        self.last_seen = 0
        self.uid: Optional[UUID] = None

    def __str__(self):
        return f"CourseObject({self.x}, {self.y}, {self.z})"


class Shape(CourseObject):
    def __init__(
        self, x: "numeric", y: "numeric", z: "numeric", shape: Shapes, color: BuoyColors
    ):
        self.x = x
        self.y = y
        self.z = z
        self.shape = shape
        self.color = color
        self.last_seen = 0
        self.uid: Optional[UUID] = None

    def __str__(self):
        return f"Shape({self.x}, {self.y}, {self.z}, {self.shape}, {self.color})"


class Buoy(CourseObject):
    def __init__(self, x: "numeric", y: "numeric", z: "numeric", color: BuoyColors):
        self.x = x
        self.y = y
        self.z = z
        self.color = color
        self.last_seen = 0
        self.uid: Optional[UUID] = None

    def __str__(self):
        return f"Buoy({self.x}, {self.y}, {self.z})"


class PoleBuoy(Buoy):
    def __init__(
        self,
        x: "numeric",
        y: "numeric",
        z: "numeric",
        color: Literal[BuoyColors.RED, BuoyColors.GREEN],
    ):
        self.x = x
        self.y = y
        self.z = z
        self.color = color
        self.last_seen = 0
        self.uid: Optional[UUID] = None

    def __str__(self):
        return f"PoleBuoy({self.x}, {self.y}, {self.z}, {self.color})"


class BallBuoy(Buoy):
    def __init__(self, x: "numeric", y: "numeric", z: "numeric", color: BuoyColors):
        self.x = x
        self.y = y
        self.z = z
        self.color = color
        self.last_seen = 0
        self.uid: Optional[UUID] = None

    def __str__(self):
        return f"BallBuoy({self.x}, {self.y}, {self.z}, {self.color})"
