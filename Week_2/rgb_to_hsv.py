import cv2
import numpy as np
import time
from picamera2 import Picamera2

# ===== CAMERA SETUP =====
FRAME_WIDTH = 320
FRAME_HEIGHT = 240

picam2 = Picamera2()
picam2.preview_configuration.main.size = (FRAME_WIDTH, FRAME_HEIGHT)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()
time.sleep(2)

try:
    while True:
        # Capture frame from PiCamera
        frame = picam2.capture_array()

        # Region of Interest (bottom part of image)
        roi = frame[100:240, :]  # height 140, full width

        # Convert to HSV color space
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Get center coordinates of the ROI
        center_x = roi.shape[1] // 2
        center_y = roi.shape[0] // 2

        # Get HSV value at the center pixel
        h, s, v = hsv[center_y, center_x]
        print(f"Center Pixel HSV = ({h}, {s}, {v})")

        # Display ROI with a crosshair on the center
        display = roi.copy()
        cv2.circle(display, (center_x, center_y), 5, (255, 255, 255), 2)
        cv2.imshow("HSV Viewer (ROI)", display)

        # Exit with 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    picam2.stop()
    cv2.destroyAllWindows()