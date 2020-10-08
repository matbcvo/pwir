# GOALS:
# 1. Show webcam
# 2. Get colors from colors.json
# 3. Display filtered image

from imutils.video import VideoStream
import cv2
import json
import numpy as np
import imutils

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

#list of points
pts = deque(maxlen=args["buffer"])


while cap.isOpened():
    # 1. OpenCV gives you a BGR image
    _, bgr = cap.read()
    #cv2.imshow("bgr", bgr)
    
    #siin mingi jama, järgmine kord findcontours()
    #bgr = imutils.resize(bgr, height=600)
    #blurred = cv2.GaussianBlur(bgr, (11, 11), 0)
    
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
    ret, im = cv2.threshold(img_gray, 65, 255, cv2.THRESH_BINARY_INV)
    img, contours, hierarchy  = cv2.findContours(im, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print("Number of Contours found = " + str(len(contours)))
    cv2.drawContours(img, contours, -1, (0,255,255), 5)
    cv2.imshow("img", img)

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
	    # only proceed if the radius meets a minimum size
	    if radius > 10:
	    	# draw the circle and centroid on the frame,
	    	# then update the list of tracked points
	    	cv2.circle(img, (int(x), int(y)), int(radius),
	    		(0, 255, 255), 2)
	    	cv2.circle(img, center, 5, (0, 0, 255), -1)
    # update the points queue
    pts.appendleft(center)
    
    
    key = cv2.waitKey(10)

    if key & 0xFF == ord("q"):
        break

cap.release()
