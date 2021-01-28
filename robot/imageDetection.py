import cv2
from UtilityFunctions import readin
import numpy as np


def contourify(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.medianBlur(mask, 13)

    maskBlue = cv2.inRange(hsv, basketLower, basketUpper)
    maskBlue = cv2.morphologyEx(maskBlue, cv2.MORPH_CLOSE, kernelBasket)
    maskBlue = cv2.morphologyEx(maskBlue, cv2.MORPH_OPEN, kernelBasket)
    maskBlue = cv2.medianBlur(maskBlue, 13)
    maskCombo = cv2.add(mask, maskBlue)
    # maskBlue = cv2.GaussianBlur(maskBlue, (5, 5), 0)

    cv2.imshow("combo", maskCombo)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    cntsPurple = cv2.findContours(maskBlue, cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)[-2]

    return cnts, cntsPurple