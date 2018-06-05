import rospy
import numpy as np
import argparse
import math
import time #no se para que
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
from controller import PID
from numpy.linalg import inv

def calculateRouteLin(pelota, anglePelota):
	x2 = pelota.x - np.cos(anglePelota)*0.2
	y2 = pelota.y - np.sin(anglePelota)*0.2
	theta2 = atan2(y2, x2)
	m1 = y2/x2
	def cuadratic(x):	
		#return a*x**2+b*x+c
		return m1*x
	return cuadratic

def calculateRoute(pelota, anglePelota, goal,theta):
	x1 = 0.2*np.cos(theta)
	y1 = 0.2*np.sin(theta)
	x2 = pelota.x - np.cos(anglePelota)*0.2
	y2 = pelota.y - np.sin(anglePelota)*0.2
	
	m1 = y1/x1
	m2 = (pelota.y-y2)/(pelota.x-x2)

	b2 = y2-m2*x2
	v = np.matrix((m1,m2,y1,y2))
	v = np.transpose(v)
	A = np.matrix(((3*x1**2, 2*x1, 1, 0), (3*x2**2, 2*x2, 1, 0), (x1**3, x1**2, x1, 1),(x2**3, x2**2, x2, 1)))
	Ai =inv(A)
	print Ai
	print v
	y = Ai*v
	print (y, m1, m2, b2, x1, y1, x2, y2)
	def cuadratic(x):	
		#return a*x**2+b*x+c
		return y[0]*x**3+y[1]*x**2+y[2]*x+y[3]
	return cuadratic

def radians2grades(radians):
	return radians*180/pi


pelota = Point()
pelota.x = 2
pelota.y = -1
anglePelota = 0
theta = math.atan2(pelota.y,pelota.x)
goal = Point()
goal.x = 4
goal.y = -1
anglePelota = atan2((goal.y-pelota.y),(goal.x-pelota.x))
x1 = (0.2*np.cos(theta))
x2 = (pelota.x - np.cos(anglePelota)*0.2)
x3 = 
print(pelota,radians2grades(anglePelota),radians2grades(theta))
#xRoute = np.linspace(x1,x2,num=10)
dx = (x2-x1)/10
print (x1, x2)
xRoute=np.array((x1, x1+dx, x1+2*dx, x1+3*dx, x1+4*dx, x1+5*dx, x1+6*dx, x1+7*dx, x1+8*dx, x1+9*dx, x2))
yRoute = calculateRoute(pelota,anglePelota,goal,theta)(xRoute)
route = np.vstack((xRoute, yRoute))
#dxlin = x2/10
xRouteLin = np.linspace(0, x2, num=10)
yRouteLin = calculateRouteLin(pelota,anglePelota)(xRouteLin)
routeLin = np.vstack((xRouteLin, yRouteLin))
print(route, routeLin)
