import cv2
import numpy as np
from numpy.linalg import norm
import time

simple_wb = cv2.xphoto.createSimpleWB()


def adjust_gamma(image: np.ndarray, gamma: float = 0.5) -> np.ndarray:
    """
    Apply gamma correction to the image to adjust brightness.
    A gamma value less than 1 brightens the image.
    
    :param image: The input image (8-bit)
    :param gamma: The gamma correction value (default: 0.5)
    :return: The gamma corrected image
    """
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255 
                      for i in range(256)]).astype("uint8")
    return cv2.LUT(image, table)

def preprocess(image: np.ndarray) -> np.ndarray:
    """
    Preprocesses an image for buoy recognition, including white balancing,
    brightness enhancement via gamma correction, contrast adjustment, and sharpening.
    
    :param image: The original image (BGR format) from the D435 camera
    :return: The preprocessed image
    """
    # 1. Automatic white balance
    image = simple_wb.balanceWhite(image)
    
    # 2. Brightness enhancement using gamma correction
    image = adjust_gamma(image, gamma=0.5)
    
    # 3. Adjust brightness and contrast further based on image brightness
    # Calculate average brightness (using norm to consider all channels)
    image_brightness = np.average(np.linalg.norm(image, axis=2)) / np.sqrt(3)
    beta = 130 - image_brightness
    image = cv2.convertScaleAbs(image, alpha=1.05, beta=beta)
    
    # 4. Sharpen the image
    blurred = cv2.GaussianBlur(image, (0, 0), 3)
    image = cv2.addWeighted(image, 1.5, blurred, -0.5, 0)
    
    return image


if __name__ == "__main__":
    cap = cv2.VideoCapture("video.mp4")

    average_preprocess_time = 0
    i = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        start = time.perf_counter_ns()
        frame = preprocess(frame)
        end = time.perf_counter_ns()

        average_preprocess_time += end - start
        i += 1

        print(f"Average preprocess time: {(average_preprocess_time / i) / 1e6} ms")

        cv2.imshow("result", frame)

        c = cv2.waitKey(1)

        if c == 27:
            break

        if cv2.getWindowProperty("result", cv2.WND_PROP_VISIBLE) < 1:
            break

        time.sleep(1 / 5)

    cap.release()
