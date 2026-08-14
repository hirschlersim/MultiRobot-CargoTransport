#!/usr/bin/env python3
"""Minimal standalone check that a webcam is accessible and streaming."""
import cv2

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("cannot open camera")

while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()