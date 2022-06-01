import time, cv2
from threading import Thread
from djitellopy import Tello
import matplotlib.pyplot as plt
from collections import deque
import imutils
import numpy as np
import matplotlib.image as mpimg
videoStream = True

# A range of color values is determined for the desired object
greenLower = (30, 50, 50) 
greenUpper = (80, 255, 255)
greenCode = cv2.COLOR_RGB2HSV


redLower = (161, 155, 84)
redUpper = (179,255,255)
redCode = cv2.COLOR_BGR2HSV

blueLower = (95,50,50)
blueUpper = (135,255,255)
blueCode = cv2.COLOR_BGR2HSV

"""
    This file is used only to check if the detection of the color works properly,
    The drone uses his camera in order to detect the balloon (without moving).
"""

# This function initializes tello drone and returnes reference to tello
def setupDrone():
    tello = Tello() # get reference
    tello.connect() # send 'command' to tello (used in the protocol to start communication)
    return tello


# This function setup the stream of tello (reboot)
def setupStream(tello : Tello):
    tello.streamoff() # in case it wasn`t already off
    tello.streamon()  # enable video streaming


# This function return the mask of the balloon for the detection
def getBalloonMask(colorLower, colorUpper, frame, code = cv2.COLOR_RGB2HSV):
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, code)
    # construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
    mask = cv2.inRange(hsv, colorLower, colorUpper)
    mask = cv2.erode(mask,None, iterations=2)
    mask = cv2.dilate(mask,None, iterations=2)
    return mask

# This function helps to the detect the balloon, circle it and return the frame
def detectBalloon(frame, colorLower, colorUpper, code):
    mask = getBalloonMask(colorLower, colorUpper, frame,code)
    # find contours in the mask and initialize the current
	# (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    # only proceed if at least one contour was found
    if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
        c = max(cnts, key=cv2.contourArea)
        ((x,y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if radius > 10:
            # draw the circle and centroid on the frame,
			# then update the list of tracked points
            cv2.circle(frame,(int(x), int(y)), int(radius),(0, 255, 255), 2)
            cv2.circle(frame,center, 5, (0, 0, 255), -1)
    return frame



def displayVideo(tello : Tello):
    setupStream(tello)
    frameRead = tello.get_frame_read()
    height, width, _ = frameRead.frame.shape #get the frame config
    while videoStream:
        img = frameRead.frame
        img = cv2.resize(img, (width,height))
        img = detectBalloon(img, greenLower, greenUpper, greenCode)
        cv2.imshow("Image", img)
        if(cv2.waitKey(1) & 0xFF == ord('q')):
            break


if __name__ == "__main__":
    tello = setupDrone()
    displayVideo(tello)



"""Links that helped me"""
#https://github.com/trunc8/ball-tracking-opencv-and-ros/blob/main/src/ball_tracker.py
#https://www.bluetin.io/opencv/object-detection-tracking-opencv-python/
#https://github.com/Practical-CV/Color-Based-Ball-Tracking-With-OpenCV/blob/master/ball_tracking.py
#https://www.youtube.com/watch?v=qJd5CKlmmww
#https://www.youtube.com/watch?v=qOyhrfotoU0

#https://jckantor.github.io/CBE30338/B.02-Visual-Tracking-of-an-Object-with-a-Drone.html
#https://github.com/einnis01/CBE30338_Drone_Project

#https://github.com/atduskgreg/opencv-processing

#https://github.com/nyukhalov/tello-face-follow