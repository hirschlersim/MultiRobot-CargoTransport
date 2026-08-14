#!/usr/bin/env python3
"""Loads a sample image and displays it scaled down, for quick manual checks."""
import cv2

image_path = "/home/labuser/catkin_ws/src/detect_object/nodes/bricks.png"
img = cv2.imread(image_path)

height, width, _ = img.shape
print("Image dimensions: ", width, "x", height)

scale_x = 2.0 / width
scale_y = 2.0 / height
scaled_width = int(width * scale_x)
scaled_height = int(height * scale_y)

scaled_image = cv2.resize(img, (scaled_width, scaled_height))
cv2.imshow("Image", scaled_image)
cv2.waitKey(0)
