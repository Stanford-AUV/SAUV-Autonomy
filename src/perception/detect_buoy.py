from pathlib import Path
import cv2
import depthai
import numpy as np


H_LOW = 0
S_LOW = 128 
V_LOW = 128 
H_HIGH = 30
S_HIGH = 255
V_HIGH = 255
                           
# Pipeline tells DepthAI what operations to perform when running - you define all the resources used and flows here
pipeline = depthai.Pipeline()

# Color camera as the output
cam_rgb = pipeline.createColorCamera()
cam_rgb.setPreviewSize(300, 300)  # Frame size
cam_rgb.setInterleaved(False)

# XLinkOut for the RGB camera output
xout_rgb = pipeline.createXLinkOut()
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

# Device initialization and running pipeline
with depthai.Device(pipeline) as device:
    q_rgb = device.getOutputQueue("rgb")

    # Define the color range for detection
    lower_bound = np.array([H_LOW, S_LOW, V_LOW])  # Replace with your color's HSV lower bound
    upper_bound = np.array([H_HIGH, S_HIGH, V_HIGH])  # Replace with your color's HSV upper bound

    while True:
        in_rgb = q_rgb.tryGet()

        if in_rgb is not None:
            frame = in_rgb.getCvFrame()
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                if cv2.contourArea(contour) > MIN_AREA:  # Define MIN_AREA to filter out small detections
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

            cv2.imshow("preview", frame)

        if cv2.waitKey(1) == ord('q'):
            break

