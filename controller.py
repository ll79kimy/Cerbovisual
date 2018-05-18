import rospy
import numpy as np
import argparse
import math
from math import atan2, pi, sqrt
#import imutils
import cv2
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
from collections import deque

RGB = None
depth = None

x = 0.0
y = 0.0
theta = 0.0


def getOdom(msg):
     global x
     global y
     global theta

     x = msg.pose.pose.position.x
     y = msg.pose.pose.position.y

     orientation = msg.pose.pose.orientation
     roll, pitch, theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

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
r = rospy.Rate(10) #10hz
rospy.Subscriber("/camera/rgb/image_color", Image, getRGB)
rospy.Subscriber("/camera/depth/image", Image, getDepth)
rospy.Subscriber("/odom", Odometry, getOdom)

cmd_vel = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size = 1)
	
bridge = CvBridge()
while not rospy.is_shutdown():	
	if RGB != None and depth != None:

		colorMask1 = np.zeros(RGB.shape[0:2], dtype='uint8')
		colorMask2 = np.zeros(RGB.shape[0:2], dtype='uint8')
		segment = None		
		gray = cv2.cvtColor(RGB,cv2.COLOR_BGR2GRAY)
		#edges = cv2.Canny(gray,400,400/2,apertureSize = 3)

				
		# Convert BGR to HSV
	  	hsv = cv2.cvtColor(RGB, cv2.COLOR_BGR2HSV)

		# define range of yellow color in HSV
		lower_yel = np.array([10,100,100])
		upper_yel = np.array([30,255,255])
		

		# Threshold the HSV image to get only yellow colors
		colorMask = cv2.inRange(hsv, lower_yel, upper_yel)
		colorMask = cv2.erode(colorMask, None, iterations=2)
		colorMask = cv2.dilate(colorMask, None, iterations=2)
		#colorMask = np.array(colorMask, dtype='uint8')	
		
		cnts = cv2.findContours(colorMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
		center = None
		if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			print ('radius', radius)
			# only proceed if the radius meets a minimum size
			if radius > 10:
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv2.circle(RGB, (int(x), int(y)), int(radius),
						(0, 255, 255), 2)
				cv2.circle(RGB, center, 5, (0, 0, 255), -1)
				if radius > 100:
					colorMask1 = colorMask
				else:
					colorMask2 = colorMask
 

		
		
		
		'''speed = Twist()
		error_angle = RGB.shape[1]/2-x
		speed.angular.z = 0.02
		if  abs(error_angle) >= 10:
			if error_angle > 0:
		    		speed.angular.z = 0.3
			else:
				speed.angular.z = -0.3
		speed.linear.x = -0.1
		print(error_angle, speed.angular)
		cmd_vel.publish(speed)
    		r.sleep()'''

		#colorMask = cv2.cvtColor(colorMask, cv2.COLOR_GRAY2RGB)
		cv2.imshow("Image from my node", np.hstack([colorMask1, colorMask2]) )		
		cv2.waitKey(1)



































		

