import cv2
import numpy as np

Image = np.ndarray

def adjust_gamma(image: Image, gamma=1.2) -> Image:
    invGamma = 1.0 / gamma
    table = np.array([(i / 255.0) ** invGamma * 255 for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)

"""
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
enhanced = clahe.apply(img)

Avoids overexposure while improving visibility of bright objects.
"""


def normalize_brightness(image: Image, target_mean=128) -> Image:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mean_intensity = cv2.mean(gray)[0]
    alpha = target_mean / mean_intensity  # Scaling factor
    adjusted = cv2.convertScaleAbs(image, alpha=alpha, beta=0)
    return adjusted

"""
equalized = cv2.equalizeHist(img)
"""