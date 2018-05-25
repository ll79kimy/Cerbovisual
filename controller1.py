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


class PID:
	"""
	Discrete PID control
	"""

	def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=0.3, Integrator_min=-0.3):

		self.Kp=P
		self.Ki=I
		self.Kd=D
		self.Derivator=Derivator
		self.Integrator=Integrator
		self.Integrator_max=Integrator_max
		self.Integrator_min=Integrator_min

		self.set_point=0.0
		self.error=0.0

	def update(self,current_value):
		"""
		Calculate PID output value for given reference input and feedback
		"""

		self.error = self.set_point - current_value

		self.P_value = self.Kp * self.error
		self.D_value = self.Kd * ( self.error - self.Derivator)
		self.Derivator = self.error

		self.Integrator = self.Integrator + self.error

		if self.Integrator > self.Integrator_max:
			self.Integrator = self.Integrator_max
		elif self.Integrator < self.Integrator_min:
			self.Integrator = self.Integrator_min

		self.I_value = self.Integrator * self.Ki

		PID = self.P_value + self.I_value + self.D_value

		return PID

	def setPoint(self,set_point):
		"""
		Initilize the setpoint of PID
		"""
		self.set_point = set_point
		self.Integrator=0
		self.Derivator=0

	def setIntegrator(self, Integrator):
		self.Integrator = Integrator

	def setDerivator(self, Derivator):
		self.Derivator = Derivator

	def setKp(self,P):
		self.Kp=P

	def setKi(self,I):
		self.Ki=I

	def setKd(self,D):
		self.Kd=D

	def getPoint(self):
		return self.set_point

	def getError(self):
		return self.error

	def getIntegrator(self):
		return self.Integrator

	def getDerivator(self):
		return self.Derivator



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

def promediarLista(lista):
    sum=0.0
    for i in range(0,len(lista)):
        sum=sum+lista[i]

    return sum/len(lista)

def velAngPos (velAng):
	print('velAngPos')
	if velAng < velAngMax:
		velAng = (velAng+0.01)
		print ('menor', velAng)
		return velAng

	else:
		velAng = velAngMax
		print ('equal', velAng)
		return velAng

def velAngNeg (velAng):	
	print('velAngNeg')
	if velAng > velAngMin:
		velAng = (velAng-0.01)
		print ('mayor', velAng)
		return velAng
	else:
		velAng = velAngMin
		print ('equal', velAng)
		return velAng

def velLinPos (velLin):
	print('velLinPos')
	if velLin < velLinMax:
		print ('menor', velLin)
		velLin = (velLin + 0.05)
		return velLin
	else:
		velLin = velLinMax
		print ('equal', velLin)
		return velLin

def velLinNeg (velLin):
	print('velLinNeg')
	if velLin > velLinMin:
		print ('mayor', velLin)
		velLin = (velLin-0.05)
		return velLin
	else:
		velLin = velLinMin
		print ('equal', velLin)
		return velLin
velLinMax = 0.2
velLinMin = 0
velAngMax = 0.3
velAngMin = -0.3
velLin = 0
velAng = 0

rospy.init_node("camera_proves")
r = rospy.Rate(10) #10hz
rospy.Subscriber("/camera/rgb/image_color", Image, getRGB)
rospy.Subscriber("/camera/depth/image", Image, getDepth)
rospy.Subscriber("/odom", Odometry, getOdom)

cmd_vel = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size = 1)

w, h = 1, 20
a = [[0 for x in range(w)] for y in range(h)]
d=1	
bridge = CvBridge()
while not rospy.is_shutdown():	

	value = 0
	pid = PID()
	pid.setKp(0.01)
	pid.setKd(0.01)
	pid.setKi(0.01)
	
	

	if RGB != None and depth != None:
		for i in range (0,20):
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
					#print ('radius', radius)

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

					
			colorMask = cv2.cvtColor(colorMask, cv2.COLOR_GRAY2RGB)
			#cv2.imshow("Image from my node", np.hstack([colorMask, RGB]) )		
			cv2.imshow('RGB',RGB)
			cv2.waitKey(1)
			a[i] = radius
			i = i+1	

			speed = Twist()
			error_angle = RGB.shape[1]/2-x
			speed.angular.z = 0

			if  abs(error_angle) >= 10:
				if error_angle > 0:
					#pid.setPoint(0.3)
			    		#speed.angular.z = 0.3
					velAng = velAngPos(velAng)
				else:
					#pid.setPoint(-0.3)
					#speed.angular.z = -0.3
					velAng = velAngNeg(velAng)				
				#print(pid.getPoint(), value)
				#value = pid.update(value)
				
				#speed.angular.z = value
			if len(cnts) != 0: 
				if d > 0.15:
					velLin = velLinPos(velLin)
					#speed.linear.x = 0.2
				else:
					velLin = velLinNeg(velLin)
					#speed.linear.x = 0
			else:
				velLinNeg(velLin)
				#speed.linear.x = 0'''
			print('angle', error_angle, velAng, 'distance', d, velLin)
			speed.angular.z=velAng
			speed.linear.x = velLin
			cmd_vel.publish(speed)
			#print(error_angle, speed.angular, speed.linear.x)

		'''r = promediarLista(a)	
		d = 60/r
		#print d
		if d > 2.5:
			d = d+0.1*(d-1)+0.04*(d-1)*(d-1)*(d-1)
		elif d < 0.2:
			d = d+0.1*(d-1)
		else:
			d = d+0.1*(d-1)+0.05*(d-1)*(d-1)*(d-1)
		
		#print r
		#print ('distance', d)

    		#r.sleep()'''

































		

