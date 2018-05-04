import rospy
import numpy as np
import argparse
import imutils
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

rospy.init_node("camera_proves")
rospy.Subscriber("/camera/rgb/image_color", Image, getRGB)
rospy.Subscriber("/camera/depth/image", Image, getDepth)
	
bridge = CvBridge()
while not rospy.is_shutdown():	
	if RGB != None and depth != None:
		
		gray = cv2.cvtColor(RGB,cv2.COLOR_BGR2GRAY)
		#edges = cv2.Canny(gray,400,400/2,apertureSize = 3)
		circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,200,  
param1=40,param2=85,minRadius=0,maxRadius=300)
		#mask = np.zeros(RGB.shape, dtype='uint8')
		#detector = RGB.copy()	
		detector = np.zeros(RGB.shape, dtype='uint8')	

		if circles != None and len(circles) > 0:
			circles = np.uint16(np.around(circles))
			mask = np.zeros(RGB.shape, dtype='uint8')
			for i in circles[0,:]:				
				cv2.circle(mask,(i[0],i[1]),i[2],(1,1,1),-1)
				#mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
				#mask = cv2.resize(mask,(480,360), interpolation = cv2.INTER_LANCZOS4  )
				#print(RGB.shape, depth.shape)			
				detector = mask*RGB				
				mean, dev = cv2.meanStdDev(detector)
				#if len(circles[0,:]) > 1:
					#print(mean,dev)
			if len(circles[0,:]) > 1:
				print(circles.shape)
		cv2.imshow("Image from my node", detector)
		
		cv2.waitKey(1)






































		

