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
import _thread
import pyqtgraph as pg
robot_enable = True

cap = cv2.VideoCapture(4)

###MAINBOARDIGA SUHTLEMISE LOOGIKA VIGANE; KIRJUTAN LIIGA KIIRESTI, MAINBOARD CONNECTION LÄHEB KATKI. VÕI SIIS while ser.inWaiting():

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

def omni_move(m1, m2, m3):
    motor1 = m1 #parem ratas
    motor2 = m2 #tagumine
    motor3 = m3 #vasak
    thrower_speed = 3000
    thrower_angle = 2700
    try:
        ser = serial.Serial(
            port='/dev/ttyACM0',
            baudrate=115200,
            parity=serial.PARITY_NONE,
            bytesize=serial.EIGHTBITS
        )
        if(ser.isOpen()):
            data = struct.pack('<3h3H', motor1, motor2, motor3, thrower_speed, thrower_angle, 0xAAAA)
            ser.write(data)
            ser.close()
        else:
            print("Serial is not open, cannot send data...")
    except serial.SerialException as error:
        print(error)

def stop():
    omni_move(0, 0, 0)
    
def right():
    omni_move(5, 5, 5)
    
    
rightWheelAngle = 120
leftWheelAngle = 240
rearWheelAngle = 0

def toBall(ball_y, ball_x):
    #robotSpeed = sqrt(robotSpeedX * robotSpeedX + robotSpeedY * robotSpeedY)
    robotSpeed = 10
    ball_x = ball_x - 320
    robotDirectionAngle = math.degrees(math.atan2(ball_y, ball_x))
    print("robotDirectionAngle: ",robotDirectionAngle)
    #640x400ish
    rightWheelLinearVelocity = robotSpeed * math.cos(math.radians(robotDirectionAngle - rightWheelAngle))
    leftWheelLinearVelocity = robotSpeed * math.cos(math.radians(robotDirectionAngle - leftWheelAngle))
    rearWheelLinearVelocity = robotSpeed * math.cos(math.radians(robotDirectionAngle - rearWheelAngle))
    print("rightWheelLinearVelocity = " + str(rightWheelLinearVelocity))
    print("leftWheelLinearVelocity = " + str(leftWheelLinearVelocity))
    print("rearWheelLinearVelocity = " + str(rearWheelLinearVelocity))
    print(f"sd:"+str(int(leftWheelLinearVelocity))+":"+str(int(rightWheelLinearVelocity))+":"+str(int(rearWheelLinearVelocity))+"\n")
    #ser.write(str(f"sd:"+str(int(rightWheelLinearVelocity))+":"+str(int(leftWheelLinearVelocity))+":"+str(int(rearWheelLinearVelocity))+"\n").encode())
    if enable_robot == False:
        return
    omni_move(int(leftWheelLinearVelocity), int(rightWheelLinearVelocity), int(rearWheelLinearVelocity))

def processImage():
    global cap
    # Start video capture
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    #cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    #print("kumb")
    #cap.set(cv2.CAP_PROP_EXPOSURE , 0)
    cam_heating_timer = 0
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
        
    #rotated_image = cv2.rotate(filtered_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    #cv2.imshow("filtered", filtered_image)
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
            toBall(center[1], center[0])
                    
        if (int(len(contours)) == 0):
            right()
    #cv2.imshow("img", img)
    cv2.imshow("filtered", filtered_image)


def fastWorker():
    global robot_enable
    print("Starting fastWorker in a separate thread")
    while running:
        
        time.sleep(0.02) # Limit control thread to 50 Hz
    # Stop the robot when not running any more
    print("STOPPING fastWorker")
    sys.exit(0)        

def slowWorker():
    processImage()

# This function will be called when CTRL+C is pressed
def signal_handler(sig, frame):
    print('\nYou pressed Ctrl+C! Closing the program nicely :)')
    global running
    running = False
    robot.stop()
    try:
        ser.close()
    except Exception:
        pass
    sys.exit(0)

running = True

_thread.start_new_thread(fastWorker, ()) # Start fastWorker in a separate thread.

# Create timer and connect it to slowWorker.
# Effectively, slowWorker() will be called no more than 10 times per second.
timer = pg.QtCore.QTimer()
timer.timeout.connect(slowWorker)
timer.start(100)

# Execute the Qt application. This function is blocking until the user closes the window.
if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
    QtGui.QApplication.instance().exec_()

# The window has been closed. Stop whatever we were doing.
running = False # Stop fastWorker
timer.stop()    # Stop slowWorker

cap.release()
ser.close()
