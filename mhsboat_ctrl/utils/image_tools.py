import cv2
import numpy as np
from numpy.linalg import norm
import time

simple_wb = cv2.xphoto.createSimpleWB()

def preprocess(image: np.ndarray) -> np.ndarray:
    """
    Preprocesses an image for buoy recognition

    :param image: The image to preprocess
    :type image: class:`numpy.ndarray`
    :return: The preprocessed image
    :rtype: class:`numpy.ndarray`
    """
    # automatic color balance
    image = simple_wb.balanceWhite(image)

    # equalize histogram
    image = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)
    channels = cv2.split(image)
    cv2.equalizeHist(channels[0], channels[0])
    cv2.merge(channels, image)
    image = cv2.cvtColor(image, cv2.COLOR_YCrCb2BGR)

    image_brightness = np.average(norm(image, axis=2)) / np.sqrt(3)

    beta = 130 - image_brightness

    # constrast and brightness
    image = cv2.convertScaleAbs(image, alpha=1.05, beta=beta)

    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

    # increase saturation
    hsv[...,1] = hsv[...,1] * 1.1

    hsv[...,1] = np.clip(hsv[...,1],0,255)

    image = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)

    # sharpen
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

        time.sleep(1/5)

    cap.release()