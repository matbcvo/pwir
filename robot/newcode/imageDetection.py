import cv2
#import pyrealsense2 as rs
import numpy as np
import math
import json
import imutils
from collections import deque
from imutils.video import VideoStream
#import threading

try:
    with open("colors.json", "r") as f:
        saved_colors = json.loads(f.read())
except FileNotFoundError:
    saved_colors = {}

print("Saved colors: ", saved_colors)

cap = cv2.VideoCapture(4)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

cam_heating_timer = 0
ball_x = 0
ball_y = 0

running = True

def imageDetect():
    #while running:
    while cap.isOpened():
        """global cam_heating_timer
        global ball_x
        global ball_y"""
        # 1. OpenCV gives you a BGR image
        _, bgr = cap.read()
        
        #autowhitebalance ja autoexposure maha!

        # 2. Convert BGR to HSV where color distributions are better
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        #cv2.imshow("hsv", hsv)
        # 3. Use filters on HSV image
        mask = np.zeros((height, width), np.uint8)

        for color, filters in saved_colors.items():
            color_mask = cv2.inRange(hsv, tuple(filters["min"]), tuple(filters["max"]))
            mask = cv2.bitwise_or(mask, color_mask)
            
        filtered_image = cv2.bitwise_or(bgr, bgr, mask=mask)
        
        #rotated_image = cv2.rotate(filtered_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        #cv2.imshow("filtered", filtered_image)
        blurred = cv2.GaussianBlur(filtered_image, (11, 11), 0)
        img_gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        #ret, im = cv2.threshold(filtered_image, 65, 255, cv2.THRESH_BINARY_INV)
        img, contours, hierarchy  = cv2.findContours(img_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print("Number of Contours found = " + str(len(contours)))
        cv2.drawContours(img, contours, -1, (0,255,255), 5)
        
        if len(contours) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            #print("This is the center:", center)
            # only proceed if the radius meets a minimum size
            if radius > 7:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(filtered_image, (int(x), int(y)), int(radius),
                        (0, 255, 255), 2)
                cv2.circle(filtered_image, center, 5, (0, 0, 255), -1)
                #give some time for camera to "warm up"
                if cam_heating_timer > 100:
                    print("lezgo")
                    ball_x = center[0]
                    ball_y = center[1]
                    #toBall(center[1], center[0])
        
        #cv2.imshow("filtered", filtered_image)
        #cv2.imshow("hsv", hsv)
        cam_heating_timer += 1
        
        key = cv2.waitKey(10)
        if key & 0xFF == ord("q"):
            break
            
    cap.release()
    cv2.destroyAllWindows()
