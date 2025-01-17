import math
import numpy as np

from mhsboat_ctrl.utils.custom_types import numeric
from mhsboat_ctrl.course_objects import Buoy

def angle(x1: numeric, y1: numeric, x2: numeric, y2: numeric, x3: numeric, y3: numeric) -> float:
    """
    Calculate the angle between 3 points in 2d space
    """
    v1 = np.array([x1 - x2, y1 - y2])
    v2 = np.array([x3 - x2, y3 - y2])
    angle = np.arctan2(np.linalg.det([v1, v2]), np.dot(v1, v2))
    return abs(np.degrees(angle))

def distance(x1: numeric, y1: numeric, x2: numeric, y2: numeric) -> float:
    """
    Calculate the distance between two points in 2d space
    """
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def midpoint(x1: numeric, y1: numeric, x2: numeric, y2: numeric) -> tuple[numeric, numeric]:
    """
    Calcualte the midpoints between two points in 2d space
    """
    return ((x1+x2)/2, (y1+y2)/2)

def calculate_buoy_angle(b1: Buoy, b2: Buoy, b3: Buoy) -> float:
    return angle(b1.x, b1.y, b2.x, b2.y, b3.x, b3.y)
