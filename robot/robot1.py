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
import struct
from MB import MB_Comm
import pyrealsense2 as rs
from enum import Enum, unique, auto
from datetime import datetime
from typing import Tuple, Union
import time


@unique
class STATE(Enum):
    EMPTYING = auto()
    WAITING = auto()
    FINDING_BALL = auto()
    DRIVING_TO_BALL = auto()
    PICKING_UP_BALL = auto()
    FINDING_BASKET = auto()
    DRIVING_TO_BASKET = auto()
    STARTING_THROWER = auto()
    THROWING_BALL = auto()


class RobotState:
    current: STATE = STATE.FINDING_BALL
    target_x: int = 0
    target_y: int = 0
    timer: Union[datetime, None] = 0

    def change_state(self, next_state):
        self.current = next_state
        self.timer = None

    def timer_ms_passed(self):
        if self.timer is None:
            self.timer = datetime.now()
        return (datetime.now() - self.timer).microseconds/1000000

    def timer_seconds_passed(self):
        if self.timer is None:
            self.timer = datetime.now()
        return (datetime.now() - self.timer).seconds


CAM_WIDTH = 640
CAM_HEIGHT = 480
##PILT ON 640x320
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

profile = pipeline.start(config)

rgb_sensor = profile.get_device().first_color_sensor()
rgb_sensor.set_option(rs.option.enable_auto_exposure, False)
rgb_sensor.set_option(rs.option.enable_auto_white_balance, False)
rgb_sensor.set_option(rs.option.auto_exposure_priority, False)
rgb_sensor.set_option(rs.option.brightness, 0)
rgb_sensor.set_option(rs.option.contrast, 50)
rgb_sensor.set_option(rs.option.exposure, 166)
rgb_sensor.set_option(rs.option.gain, 64)
rgb_sensor.set_option(rs.option.gamma, 300)
rgb_sensor.set_option(rs.option.hue, 0)
rgb_sensor.set_option(rs.option.saturation, 64)
rgb_sensor.set_option(rs.option.sharpness, 50)
rgb_sensor.set_option(rs.option.white_balance, 4600)

def getFrames(pipeline):
    frames = pipeline.wait_for_frames()
    return frames

ball_y_requirement = 340
#ser=serial.Serial('/dev/ttyACM0', 115200, timeout=0.00001)
#print(ser.name)

# Load saved color values from colors.json
try:
    with open("colors.json", "r") as f:
        saved_colors = json.loads(f.read())
except FileNotFoundError:
    saved_colors = {} 

print("Saved colors: ", saved_colors)
    
def getRadius(contours):
    c = max(contours, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    return radius

def getCenter(contours):
    if(len(contours)):
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        try:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            return center
        except ZeroDivisionError as e:
            print(e)
    return None

drive = MB_Comm()
state = RobotState()
#rfser = drive.ser
while True:
    
    #(grabbed, bgr) = cap.read()
    frames = getFrames(pipeline)
    color_frame = frames.get_color_frame()
    bgr = np.asanyarray(color_frame.get_data())

    # 1. OpenCV gives you a BGR image
    #_, bgr = cap.read()q
    
    #autowhitebalance ja autoexposure maha!
    
    #blurred = cv2.gaussianBlur(bgr, (11, 11), 0)
    # 2. Convert BGR to HSV where color distributions are better
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    #cv2.imshow("hsv", hsv)
    # 3. Use filters on HSV image
    kernel = np.ones((3, 3), np.uint8)

    print(saved_colors["green"]["min"])
    print(saved_colors["green"]["max"])
    mask = cv2.inRange(hsv, tuple(saved_colors["green"]["min"]), tuple(saved_colors["green"]["max"]))
    #mask = cv2.bitwise_or(mask, color_mask)
    
    #mask = np.zeros((height, width), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask, kernel, iterations=2)

    img, contours, hierarchy  = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #cv2.imshow("blurred",blurred)
    print("Number of Contours found = " + str(len(contours)))
    #cv2.drawContours(img, contours, -1, (0,255,255), 5)
    
    
    bluemask = cv2.inRange(hsv, tuple(saved_colors["blue"]["min"]), tuple(saved_colors["blue"]["max"]))
    bluemask = cv2.morphologyEx(bluemask, cv2.MORPH_CLOSE, kernel)
    bluemask = cv2.morphologyEx(bluemask, cv2.MORPH_OPEN, kernel)

    bluemask = cv2.dilate(bluemask, kernel, iterations=2)
    
    bluecontours  = cv2.findContours(bluemask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    print("Number of Contours found = " + str(len(bluecontours)))
    
    if state.current is STATE.FINDING_BALL:
        if len(contours) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            try:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            except ZeroDivisionError as e:
                print(e)
                
            print("This is the center:", center)
            # only proceed if the radius meets a minimum size
            if radius > 0.1:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(bgr, (int(x), int(y)), int(radius),
                        (0, 255, 255), 2)
                cv2.circle(bgr, center, 5, (0, 0, 255), -1)
                #give some time for camera to "warm up"
                print("lezgo")
                #               y           x
                if drive.ballMiddle(center[0], CAM_WIDTH):
                    drive.toBall(center[1], center[0])
                elif center[0] > (380 + 10):
                    #pall on paremal, pöörata paremale aeglaselt
                    drive.omni_move(0, 0, 3)
                elif center[0] < (380 - 10):
                    #pall on vasakul, pöörata vasakle aeglaselt
                    drive.omni_move(3, 0, 0)
                #if center[1] > CAM_HEIGHT-180 and (drive.ballMiddle(center[0], CAM_WIDTH)):
                if center[1] > CAM_HEIGHT-100 and (drive.ballMiddle(center[0], CAM_WIDTH)):
                    print('found ball')
                    state.change_state(STATE.PICKING_UP_BALL)
                    drive.stop()
                    time.sleep(1.2)
                #elif center[1] > CAM_HEIGHT-180 and (drive.ballMiddle(center[0], CAM_WIDTH) == False):
                #    drive.right() #tglt left #panin selle sp et ta jäi lopmatuseni taga ajama oma nina korvalt palle
                else:
                    #print("nothing")
                    drive.toBall(center[1], center[0])
                cv2.line(bgr, (center[0], 0), (center[0], 480), (255, 0, 0), 1)
                cv2.line(bgr, (380, 0), (380, 480), (0, 255, 0), 1)
                cv2.line(bgr, (0, center[1]), (640, center[1]), (0, 0, 255), 1)
                cv2.line(bgr, (0, 320), (640, 320), (0, 255, 255), 1)
        else:
            drive.right()
    elif state.current is STATE.PICKING_UP_BALL:
        print("picking up the ball")
        #taski jaoks
        if center[1] > CAM_HEIGHT-220 and (drive.ballMiddle(center[0], CAM_WIDTH)):
            drive.fwd()
            time.sleep(1.6)
            drive.stop()
            state.change_state(STATE.FINDING_BASKET)
            
        
        
        #drive.stop()
    elif state.current is STATE.FINDING_BASKET:
        print("finding the basket")
        """print(saved_colors["blue"]["min"])
        print(saved_colors["blue"]["max"])
        bluemask = cv2.inRange(hsv, tuple(saved_colors["blue"]["min"]), tuple(saved_colors["blue"]["max"]))
        bluemask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        bluemask = cv2.dilate(mask, kernel, iterations=2)
    
        img, bluecontours, hierarchy  = cv2.findContours(bluemask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print("Number of Contours found = " + str(len(bluecontours)))"""
        
        if len(bluecontours) > 0: #siis kui näen korvi?
            #drive.toBall2(center[1], center[0])
            for contour in bluecontours:
                (x,y,w,h) = cv2.boundingRect(contour)
                cv2.rectangle(bluemask, (x,y), (x+w,y+h), (255,0,0), 2)
                print("seal sa oledki raisk")
                if drive.basketMiddle(x) == True:
                    drive.stop()
                    drive.startThrower()
                    time.sleep(1)
                    drive.stopThrower()
                    state.change_state(STATE.FINDING_BALL)
##            c = max(contours, key=cv2.contourArea)
            
##            ((x, y), radius) = cv2.minEnclosingCircle(c)
##            M = cv2.moments(c)
##            try:
##                bluecenter = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
##            except ZeroDivisionError as e:
##                print(e)
##            print("This is the basket center :", bluecenter)
        else:
            drive.omni_move(3, -12, 0)
            print("tiirutan")
            
##            if (drive.basketMiddle(bluecenter[0], CAM_WIDTH)) & (drive.ballMiddle(center[0], CAM_WIDTH)):
##                print("asun viskama")
##                drive.stop()
                
    else:
        print("miski läks nihu")
                 
            
                
            
    #KUI PALL ON KÄES
    #elif state.current is STATE.FINDING_BASKET:
            
    
                
    #cv2.imshow("img", img)
    
    
    
    cv2.imshow("bgr", bgr)
    cv2.imshow("mask", mask)
    cv2.imshow("bluemask", bluemask)

    #cam_heating_timer += 1
    
    key = cv2.waitKey(10)
    if key & 0xFF == ord("q"):
        drive.stop()
        drive.running = False
        pipeline.stop()
        break
drive.running = False
drive.stop()
#cap.release()
pipeline.stop()

