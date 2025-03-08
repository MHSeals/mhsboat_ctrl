# import math
import numpy as np
from typing import Tuple

def dist(x1: float, y1: float, x2: float, y2: float) -> float:
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** (1 / 2)

def mdpt(x1: float, y1: float, x2: float, y2: float) -> Tuple[float, float]:
    return ((x1 + x2) / 2, (y1 + y2) / 2)

def p2a(
    intrinsics: Tuple[float, float, float, float],  # (fx, fy, cx, cy)
    pixel1: Tuple[int, int],  # (u1, v1) - Reference point (typically center of the image)
    pixel2: Tuple[int, int]   # (u2, v2) - Target pixel
) -> float:
    """
    Computes the signed angle needed to turn toward a target pixel from a reference pixel.

    Parameters:
    - intrinsics: (fx, fy, cx, cy) - Camera intrinsic parameters
    - pixel1: (u1, v1) - Reference pixel (usually the center of the image)
    - pixel2: (u2, v2) - Target pixel

    Returns:
    - Signed angle in degrees (left is positive, right is negative)
    """
    # Unpack intrinsics
    fx, fy, cx, cy = intrinsics

    # Convert to normalized camera coordinates
    X1, Y1 = (pixel1[0] - cx) / fx, (pixel1[1] - cy) / fy
    X2, Y2 = (pixel2[0] - cx) / fx, (pixel2[1] - cy) / fy

    # 3D direction vectors in camera space
    v1 = np.array([X1, Y1, 1], dtype=np.float64)  # Reference (default: looking straight)
    v2 = np.array([X2, Y2, 1], dtype=np.float64)  # Target

    # Compute the cosine of the angle using the dot product
    cos_theta = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    cos_theta = np.clip(cos_theta, -1.0, 1.0)  # Clip for numerical stability
    theta = np.arccos(cos_theta)  # Angle in radians

    # Compute the sign using the cross product (to determine left or right turn)
    cross_product = np.cross(v1, v2)
    sign = np.sign(cross_product[1])  # Positive if left, negative if right

    return np.degrees(theta) * sign  # Return signed angle in degrees