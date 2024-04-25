"""
   Name : Abdelrahman Amr Abdelrahman El Sayed 
"""

import cv2
import numpy as np

# Load the image
image = cv2.imread("/Users/python/Desktop/abdelrahman_python_task_6_robotics/color_corrected_images/15.jpeg")

# Convert to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Thresholding to isolate red color
lower_red = np.array([0, 0, 100])
upper_red = np.array([100, 100, 255])
mask = cv2.inRange(image, lower_red, upper_red)

# Edge detection
edges = cv2.Canny(mask, 100, 200)

# Contour detection
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Measure thickness
if len(contours) > 0:
    outer_contour = max(contours, key=cv2.contourArea)
    inner_contour = min(contours, key=cv2.contourArea)
    thickness = cv2.arcLength(outer_contour, True) - cv2.arcLength(inner_contour, True)
    print(f"The thickness of the red stick is approximately {thickness} pixels.")
else:
    print("No contours found.")
