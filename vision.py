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
		segment = None		
		gray = cv2.cvtColor(RGB,cv2.COLOR_BGR2GRAY)
		#edges = cv2.Canny(gray,400,400/2,apertureSize = 3)
		circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,200,  
param1=40,param2=60,minRadius=0,maxRadius=300)
			
		
		if circles != None and len(circles) > 0:
			circles = np.uint16(np.around(circles))
			
		
			
			for i in circles[0,:]:
				mask = np.zeros(RGB.shape, dtype='uint8')				
				cv2.circle(mask,(i[0],i[1]),i[2],(1,1,1),-1)
				#mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
				#mask = cv2.resize(mask,(480,360), interpolation = cv2.INTER_LANCZOS4  )
				#print(RGB.shape, depth.shape)			
				detector = mask*RGB
				nPixelsCircle= np.sum (mask)/3

				
				# Convert BGR to HSV
			  	hsv = cv2.cvtColor(detector, cv2.COLOR_BGR2HSV)

				# define range of yellow color in HSV
				lower_yel = np.array([13,100,100])
				upper_yel = np.array([20,255,255])
				

				# Threshold the HSV image to get only yellow colors
				colorMask = cv2.inRange(hsv, lower_yel, upper_yel)
				nPixelsColor= np.sum (colorMask)/255
				colorEstimator = nPixelsColor/nPixelsCircle
				if colorEstimator > 0.8:
					segment = mask*RGB	
			
		if segment == None:
		#if no circle is detected, turn in direcction of the yellow or left
	
			A = np.zeros((RGB.shape[0],int(RGB.shape[1]/2),3), dtype='uint8')	
			l = np.concatenate((RGB[:,0:int(RGB.shape[1]/2)], A),axis=1)
			r = np.concatenate((A, RGB[:,int(RGB.shape[1]/2):RGB.shape[1]]),axis=1)
			segmentl, nPixelsl= detectColor(l)
			segmentr, nPixelsr= detectColor(r)
			print(nPixelsr,nPixelsl)
			if nPixelsl > 1.1*nPixelsr:
				segment = segmentl
			elif nPixelsr > 1.1*nPixelsl:
				segment = segmentr
			else:
				segment = RGB
		#cv2.imshow("images", np.hstack([segmentl, segmentr,segment]))
		cv2.imshow("Image from my node", segment )		
		cv2.waitKey(1)

			







































		

