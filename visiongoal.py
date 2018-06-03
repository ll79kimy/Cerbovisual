import rospy
import numpy as np
import argparse
import math
#import imutils
import cv2
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import deque

RGB = None
depth = None

def getRGB(data):
	global RGB
	try:
		RGB = bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)

def getDepth(data):
	global depth
	try:
		depth = bridge.imgmsg_to_cv2(data, "32FC1")
	except CvBridgeError as e:
		print(e)

def interThreshold(img,x,y):
	ret,thresh = cv2.threshold(img,x,255,3)
	ret,thresh = cv2.threshold(thresh,y,255,4)
	return thresh

def binary(img):
	ret,thresh = cv2.threshold(img,0,255,0)
	return thresh

def detectColor(img):
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	# define range of yellow color in HSV
	lower_yel = np.array([13,100,100])
	upper_yel = np.array([20,255,255])
	
	# Threshold the HSV image to get only yellow colors
	colorMask = cv2.inRange(hsv, lower_yel, upper_yel)
	segment = cv2.cvtColor(colorMask, cv2.COLOR_GRAY2RGB)
	return segment, np.sum(colorMask/255)


rospy.init_node("camera_proves")
rospy.Subscriber("/camera/rgb/image_color", Image, getRGB)
rospy.Subscriber("/camera/depth/image", Image, getDepth)
	
bridge = CvBridge()
while not rospy.is_shutdown():	
	if RGB != None and depth != None:
		# Convert BGR to HSV
	  	hsv = cv2.cvtColor(RGB, cv2.COLOR_BGR2HSV)
		# define range of b color in HSV
		lower_black = np.array([0,0,0])
		upper_black = np.array([255,255,0])
	        # define range of blue color in HSV
	        lower_blue = np.array([110,50,50])
	        upper_blue = np.array([120,255,255])
		# define range of red color in HSV
		lower_red = np.array([169, 100, 100], dtype=np.uint8)
		upper_red = np.array([189, 255, 255], dtype=np.uint8)
		# Threshold the HSV image to get only black colors
		colorMask = cv2.inRange(hsv, lower_black, upper_black)
		colorMask = cv2.erode(colorMask, None, iterations=2)
		colorMask = cv2.dilate(colorMask, None, iterations=2)
		# Threshold the HSV image to get only blue colors
		mask = cv2.inRange(hsv, lower_red, upper_red)  
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)  
		#cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
		# Bitwise-AND mask and original image
    		res = cv2.bitwise_and(RGB,RGB, mask= mask)
    		#contours,hierarchy = cv2.findContours(mask, 1, 2)
		#cnt = contours[0]
		cv2.imshow('res', res)
		'''i=0
		if len(cnts) > 0:
			#rectangula	
			for i in range (0,len(cnts)):
				print i
				x,y,w,h = cv2.boundingRect(cnts[i])
				cv2.rectangle(mask,(x,y),(x+w,y+h),(0,0,255),2)	
				cv2.imshow('Mask',mask)
				cv2.waitKey(1)
				i=i+1
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			#print ('radius', radius)

			# only proceed if the radius meets a minimum size
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(RGB, (int(x), int(y)), int(radius),
					(0, 255, 255), 2)
			cv2.circle(RGB, center, 5, (0, 0, 255), -1)
			cv2.imshow('RGB',RGB)
			print len(cnts)'''


   

