import numpy as np
from typing import Union

numeric = Union[int, float, complex]
def angle(x1: numeric, y1: numeric, x2: numeric, y2: numeric, x3: numeric, y3: numeric) -> float:
    """
    Calculate the angle between 3 points in 2d space
    """
    v1 = np.array([x1 - x2, y1 - y2])
    v2 = np.array([x3 - x2, y3 - y2])
    angle = np.arctan2(np.linalg.det([v1, v2]), np.dot(v1, v2))
    return abs(np.degrees(angle))

print(angle(1, 0, 0, 0, 0, 1))