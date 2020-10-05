# GOALS:
# 1. Show webcam
# 2. Get colors from colors.json
# 3. Display filtered image


import cv2
import json
import numpy as np

# Load saved color values from colors.json
try:
    with open("colors.json", "r") as f:
        saved_colors = json.loads(f.read())
except FileNotFoundError:
    saved_colors = {}

print("Saved colors: ", saved_colors)


# Start video capture
cap = cv2.VideoCapture(4)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

#Set up blob detector
#detector = cv2.SimpleBlobDetector()

while cap.isOpened():
    # 1. OpenCV gives you a BGR image
    _, bgr = cap.read()
    #cv2.imshow("bgr", bgr)

    # 2. Convert BGR to HSV where color distributions are better
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    #cv2.imshow("hsv", hsv)

    # 3. Use filters on HSV image
    mask = np.zeros((height, width), np.uint8)

    for color, filters in saved_colors.items():
        color_mask = cv2.inRange(hsv, tuple(filters["min"]), tuple(filters["max"]))
        mask = cv2.bitwise_or(mask, color_mask)
    
    filtered_image = cv2.bitwise_or(bgr, bgr, mask=mask)
    cv2.imshow("image", filtered_image)

    # 4. TODO: locate ball and basket coordinates
    
    key = cv2.waitKey(10)

    if key & 0xFF == ord("q"):
        break

cap.release()
