# GOALS:
# 1. Show webcam
# 2. Filter ball color
# 2.a. Maybe try HSV instead of RGB
# 3. Done


import cv2
import json
from functools import partial
import numpy as np
import pyrealsense2 as rs


pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

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




# Load saved color values from colors.json
try:
    with open("colors.json", "r") as f:
        saved_colors = json.loads(f.read())
except FileNotFoundError:
    saved_colors = {}

print("Saved color values: ", saved_colors)
color = input("What color to threshold: ")

# Read color values from colors.json or initialize new values
if color in saved_colors:
    filters = saved_colors[color]
else:
    filters = {
        "min": [0, 0, 0], # HSV minimum values
        "max": [179, 255, 255] # HSV maximum values
    }

def save():
    saved_colors[color] = filters

    with open("/home/robot/Desktop/pwir/robot/colors.json", "w") as f:
        f.write(json.dumps(saved_colors))

def update_range(edge, channel, value):
    # edge = "min" or "max"
    # channel = 0, 1, 2 (H, S, V)
    # value = new slider value
    filters[edge][channel] = value

# Create sliders to filter colors from image
cv2.namedWindow("mask")

# createTrackbar(name, window name, initial value, max value, function to call on change)
cv2.createTrackbar("h_min", "mask", filters["min"][0], 179, partial(update_range, "min", 0))
cv2.createTrackbar("s_min", "mask", filters["min"][1], 255, partial(update_range, "min", 1))
cv2.createTrackbar("v_min", "mask", filters["min"][2], 255, partial(update_range, "min", 2))
cv2.createTrackbar("h_max", "mask", filters["max"][0], 179, partial(update_range, "max", 0))
cv2.createTrackbar("s_max", "mask", filters["max"][1], 255, partial(update_range, "max", 1))
cv2.createTrackbar("v_max", "mask", filters["max"][2], 255, partial(update_range, "max", 2))

# Start video capture
cap = cv2.VideoCapture(4)
cap.set(cv2.CAP_PROP_FPS, 60)


while True:
    # 1. OpenCV gives you a BGR image
    #_, bgr = cap.read()
    frames = getFrames(pipeline)
    color_frame = frames.get_color_frame()
    bgr = np.asanyarray(color_frame.get_data())
    cv2.imshow("bgr", bgr)
    
    kernel = np.ones((3, 3), np.uint8)

    #mask = cv2.bitwise_or(mask, color_mask)
    
    #mask = np.zeros((height, width), np.uint8)
    
    # 2. Convert BGR to HSV where color distributions are better
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv", hsv)

    # 3. Use filters on HSV image
    mask = cv2.inRange(hsv, tuple(filters["min"]), tuple(filters["max"]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask, kernel, iterations=2)
    cv2.imshow("mask", mask)

    key = cv2.waitKey(10)

    if key & 0xFF == ord("s"):
        print(tuple(filters["min"]))
        print(tuple(filters["max"]))
        save()

    if key & 0xFF == ord("q"):
        break

pipeline.stop()
#cap.release()