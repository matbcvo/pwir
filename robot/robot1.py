# GOALS:
# 1. Show webcam
# 2. Get colors from colors.json
# 3. Display filtered image
import serial
from collections import deque
from collections import deque
from imutils.video import VideoStream
import cv2
import json
import numpy as np
import imutils
import math

ball_y_requirement = 340
ser=serial.Serial('/dev/ttyACM0', 115200, timeout=0.00001)
print(ser.name)

# Load saved color values from colors.json
try:
    with open("colors.json", "r") as f:
        saved_colors = json.loads(f.read())
except FileNotFoundError:
    saved_colors = {}

print("Saved colors: ", saved_colors)
def stop():
    ser.write(f"sd:0:0:0\n".encode())

def right():
    ser.write(f"sd:5:5:5\n".encode())
    #get = ser.readline().decode()
    #print("get:", get)
    
def left():
    ser.write(f"sd:-5:-5:-5\n".encode())
    #get = ser.readline().decode()
    #print("get:", get)
    
def fwd():
    ser.write(f"sd:-10:10:0\n".encode())
    
rightWheelAngle = math.radians(120)
leftWheelAngle = math.radians(240)

def toBall(ball_y, ball_x):
    #robotSpeed = sqrt(robotSpeedX * robotSpeedX + robotSpeedY * robotSpeedY)
    robotSpeed = 1
    robotDirectionAngle = math.atan2(ball_y, ball_x)
    print("robotDirectionAngle: ",robotDirectionAngle)
    #640x400ish
    rightWheelLinearVelocity = robotSpeed * math.cos(robotDirectionAngle - rightWheelAngle)
    leftWheelLinearVelocity = robotSpeed * math.cos(robotDirectionAngle - leftWheelAngle)
    print("rightWheelLinearVelocity = " + str(rightWheelLinearVelocity))
    print("leftWheelLinearVelocity = " + str(leftWheelLinearVelocity))
    #print(f"sd:"+str(int(leftWheelLinearVelocity))+":"+str(int(rightWheelLinearVelocity))+":0\n")
    #ser.write(f"sd:"+str(int(leftWheelLinearVelocity))+":"+str(int(rightWheelLinearVelocity))+":0\n".encode())
    
    
# Start video capture
cap = cv2.VideoCapture(4)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
cam_heating_timer = 0
while cap.isOpened():
    # 1. OpenCV gives you a BGR image
    _, bgr = cap.read()
    
    # 2. Convert BGR to HSV where color distributions are better
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    #cv2.imshow("hsv", hsv)
    # 3. Use filters on HSV image
    mask = np.zeros((height, width), np.uint8)

    for color, filters in saved_colors.items():
        color_mask = cv2.inRange(hsv, tuple(filters["min"]), tuple(filters["max"]))
        mask = cv2.bitwise_or(mask, color_mask)
        
    filtered_image = cv2.bitwise_or(bgr, bgr, mask=mask)
    blurred = cv2.GaussianBlur(filtered_image, (11, 11), 0)
    img_gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    #ret, im = cv2.threshold(filtered_image, 65, 255, cv2.THRESH_BINARY_INV)
    img, contours, hierarchy  = cv2.findContours(img_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print("Number of Contours found = " + str(len(contours)))
    cv2.drawContours(img, contours, -1, (0,255,255), 5)
    
    # 4. TODO: locate ball and basket coordinates
    # only proceed if at least one contour was found
    if len(contours) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        #print("c:", c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        print("This is the center:", center)
        # only proceed if the radius meets a minimum size
        if radius > 7:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(filtered_image, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
            cv2.circle(filtered_image, center, 5, (0, 0, 255), -1)
            #give some time for camera to "warm up"
            if cam_heating_timer > 100:
                #print("lezgo")
                toBall(center[1], center[0])
                if center[0] < 350 and center[0]>330 :
                    print("Ball straight ahead!")
                    #fwd()
                    toBall(center[1],center[0])
                    if center[1] > 280:
                        stop()
                        
                elif center[0] <330:
                    print("Ball too far left")
                    left()
                elif center[0] > 350:
                    print("Ball too far right")
                    right()
                else:
                    right()
    elif cam_heating_timer > 100:
        right()
	    	
    cv2.imshow("img", img)
    cv2.imshow("filtered", filtered_image)
    get = ser.readline().decode()
    print("get:", get)
    
    
    cam_heating_timer += 1
    
    key = cv2.waitKey(10)
    if key & 0xFF == ord("q"):
        break

cap.release()
ser.close()