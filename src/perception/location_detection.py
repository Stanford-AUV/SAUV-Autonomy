import cv2
import depthai as dai
import numpy as np

# Define HSV range for the color red
H_LOW, S_LOW, V_LOW = 0, 50, 50  # Red lower bound
H_HIGH, S_HIGH, V_HIGH = 10, 255, 255  # Red upper bound

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)

xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutSpatialData = pipeline.create(dai.node.XLinkOut)
xinSpatialCalcConfig = pipeline.create(dai.node.XLinkIn)

xoutDepth.setStreamName("depth")
xoutSpatialData.setStreamName("spatialData")
xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)
stereo.depth.link(spatialLocationCalculator.inputDepth)
spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
spatialLocationCalculator.out.link(xoutSpatialData.input)
xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    spatialCalcQueue = device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)
    spatialCalcConfigInQueue = device.getInputQueue("spatialCalcConfig")

    while True:
        inDepth = depthQueue.get() # Get depth data
        depthFrame = inDepth.getFrame() # depthFrame values are in millimeters

        # Convert depth frame to a visible format and to HSV
        depthFrameColor = cv2.applyColorMap(cv2.convertScaleAbs(depthFrame, alpha=0.03), cv2.COLORMAP_JET)
        hsvFrame = cv2.cvtColor(depthFrameColor, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsvFrame, (H_LOW, S_LOW, V_LOW), (H_HIGH, S_HIGH, V_HIGH))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the largest rectangle
        largest_area = 0
        best_rect = None
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h
            if area > largest_area:
                largest_area = area
                best_rect = (x, y, w, h)

        if best_rect:
            x, y, w, h = best_rect
            topLeft = dai.Point2f(x / depthFrameColor.shape[1], y / depthFrameColor.shape[0])
            bottomRight = dai.Point2f((x + w) / depthFrameColor.shape[1], (y + h) / depthFrameColor.shape[0])
            config = dai.SpatialLocationCalculatorConfigData()
            config.depthThresholds.lowerThreshold = 100
            config.depthThresholds.upperThreshold = 10000
            config.roi = dai.Rect(topLeft, bottomRight)
            cfg = dai.SpatialLocationCalculatorConfig()
            cfg.addROI(config)
            spatialCalcConfigInQueue.send(cfg)

        # Update the display
        cv2.imshow("depth", depthFrameColor)

        if cv2.waitKey(1) == ord('q'):
            break
