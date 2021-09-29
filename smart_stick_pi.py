from picamera.array import PiRGBArray
import cv2
from picamera import PiCamera
import time
import numpy as np


w = 800
h = 608
camera = PiCamera()
camera.resolution = (w, h)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(w, h))
time.sleep(1)


for frame1 in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    frame = frame1.array
    frame = cv2.GaussianBlur(orig_frame, (5, 5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low_yellow = np.array([18, 94, 140])
    up_yellow = np.array([48, 255, 255])

    mask = cv2.inRange(hsv, low_yellow, up_yellow)

    kernelOpen = np.ones((5, 5))
    kernelClose = np.ones((20, 20))
    maskOpen = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelOpen)
    maskClose = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernelClose)

    edges = cv2.Canny(maskOpen, 75, 150)

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, maxLineGap=50)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
    else:
        print("You are out of track")

    cv2.imshow("frame", frame)
    cv2.imshow("edges", edges)

    key = cv2.waitKey(1)
    if key == ord('s'):
        break

    rawCapture.truncate(0)


cv2.destroyAllWindows()
camera.close()
