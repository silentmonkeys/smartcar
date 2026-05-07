#!/usr/bin/env python3
import cv2

cap=cv2.VideoCapture(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


frame=cap.read()[1]
cv2.imwrite("camera_test.jpg", frame)