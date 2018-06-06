import rospy
import numpy as np
import argparse
import math
import time #no se para que
from math import atan2, pi, sqrt
#import imutils
import cv2
from std_msgs.msg import Empty
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
from controller import PID
from numpy.linalg import inv
RGB = None

x = 0.0
y = 0.0
theta = 0.0
angularValue = 0

def getOdom(msg):
#function for receiving the actual position and orientation of the robot. Orientation in quateriones too roll pitch yaw angles
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    orientation = msg.pose.pose.orientation
    roll, pitch, theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])


def getRGB(data):
#function to get the acutal RGB imagen of the camera
    global RGB
    try:
        RGB = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

def promediarLista(lista):
#function to calculate the promedian of a list/array
    #sum=0.0
    #for i in range(0,len(lista)):
        #sum=sum+lista[i]

    #return sum/len(lista)
   return float(sum(lista)) / max(len(lista), 1)

'''def newOdom(msg):

	global x
	global y
	global theta

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	rot_q = msg.pose.pose.orientation
	(r, p, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	return x,y,theta'''

def posPelota (x,y,theta, distance):
#calculates the position x and y of the ball with the position of the robot, its angle and the distance between the ball and the robot
	pelota.x = x+math.cos(theta)*distance 
	pelota.y = y+math.sin(theta)*distance
	return pelota

def calculateDistance(radiusList):
#calculates the distance between ball and robot, first with a guess and despues with functions for regions because the functions of the distance is not linear
	radiusMean = promediarLista(radiusList)
	distance = 60/radiusMean
	if distance > 2.5:
		distance = distance+0.1*(distance-1)+0.04*(distance-1)*(distance-1)*(distance-1)
	elif distance < 0.2:
		distance = distance+0.1*(distance-1)
	else:
		distance = distance+0.1*(distance-1)+0.05*(distance-1)*(distance-1)*(distance-1) 
	return distance

def radians2grades(radians):
	return radians*180/pi

def calculateRoute(pelota, anglePelota, goal,theta):
#the function connects to linear splines with a kubik spline. the linear splines are between the ball and an point 30 cm in the opposite direction of the goal(point 2) and between the startpoint of the robot and 20 cm in direction of theta
	#reference points of spline in directions of goal (2) or the actual orientation of the robot (1)
	x1 = 0.2*np.cos(theta)
	y1 = 0.2*np.sin(theta)
	x2 = pelota.x - np.cos(anglePelota)*0.3
	y2 = pelota.y - np.sin(anglePelota)*0.3
	#function linear y = m*x+b
	m1 = y1/x1
	m2 = (pelota.y-y2)/(pelota.x-x2)
	b2 = y2-m2*x2

	#calculate matrix of the kubik spline a*x³+b*x²+c*x+d
	v = np.matrix((m1,m2,y1,y2))
	v = np.transpose(v)
	A = np.matrix(((3*x1**2, 2*x1, 1, 0), (3*x2**2, 2*x2, 1, 0), (x1**3, x1**2, x1, 1),(x2**3, x2**2, x2, 1)))
	Ai =inv(A)
	print ("Ai", Ai)
	print ("v", v)
	m = Ai*v
	print ("m", m,"m1", m1,"m2", m2,"b2", b2, "x1", x1,"y1", y1,"x2", x2,"y2", y2)
	
	def cuadratic(x):
		#calculate the function values of specified values of x 	
		return m[0]*x**3+m[1]*x**2+m[2]*x+m[3]
	return cuadratic

'''def calculateRouteLin(pelota, anglePelota):
	x2 = pelota.x - np.cos(anglePelota)*0.2
	y2 = pelota.y - np.sin(anglePelota)*0.2
	theta2 = atan2(y2, x2)
	m1 = y2/x2
	def cuadratic(x):	
		#return a*x**2+b*x+c
		return m1*x
	return cuadratic'''

def angleError(a, b):
    #calculate the difference between dos angles
    diff = abs(a - b);
    return diff

def searchBall():
#function to search for a yellow ball in an imagen with a minimal radius. Finally drawing a line around the ball
    global i
    global r
    if RGB != None:

        gray = cv2.cvtColor(RGB,cv2.COLOR_BGR2GRAY)

        # Convert   BGR to HSV
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
            ((rx, ry), radius) = cv2.minEnclosingCircle(c)

	    i+=1
	    radiusList.pop(0)
	    radiusList.append(radius)

            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > r:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(RGB, (int(rx), int(ry)), int(radius),(0, 255, 255), 2)
                cv2.circle(RGB, center, 5, (0, 0, 255), -1)	
		colorMask = cv2.cvtColor(colorMask, cv2.COLOR_GRAY2RGB)
		return RGB.shape[1]/2-center[0]
    return None 

#initialize the rospy node and subscribe to get the imagen and odometry
rospy.init_node("ballFollower")
r = rospy.Rate(10) #10hz
rospy.Subscriber("/camera/rgb/image_color", Image, getRGB)
rospy.Subscriber("/odom", Odometry, getOdom)

cmd_vel = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size = 1)
reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)


bridge = CvBridge()

#set initial speed to zero
speed = Twist()
speed.angular.z = 0
speed.linear.x = 0

#define the parameters of the PID controller (linear and angular)
angularController = PID()
angularController.setKp(0.001)
angularController.setKi(0.5)
angularController.setKd(0.005)

angularEntry = 0

linearController = PID()
linearController.setKp(0.001)
linearController.setKi(0.05)
linearController.setKd(0.005)

linearEntry = 0

#plot the control
plt.ion()

xline = range(100)
aline = [0]*100
bline = [0]*100
cline = [0]*100
dline = [0]*100

fig = plt.figure()

ax1 = fig.add_subplot(211)
ax1.set_ylim(-0.5, 0.5)
line1, = ax1.plot(xline, aline, 'r-')
line2, = ax1.plot(xline, bline, 'b-')


ax2 = fig.add_subplot(212)
ax2.set_ylim(-0.5, 0.5)
line3, = ax2.plot(xline, cline, 'r-')
line4, = ax2.plot(xline, dline, 'b-')
#initialization of used varibales 
i = 0
#use 20 values for the promedio calculation of the radius of the ball
radiusList = [0]*20
#the inicial distance between robot and ball is 1 m
distance = 1
#the minimal radius of the ball is 15 pixeles
r= 15

pelota = Point()
anglePelota = 0

#the goal is defines in the position 4, -1
goal = Point()
goal.x = 4
goal.y = -1

point = Point()

#variables for the sequences of the programme
searching = True
moving = False
hitting = False
route = None

#defines the number of steps in the calculation of the route
idxPoint = 0
nPoints = 5
 
reset_odom.publish(Empty())
while not rospy.is_shutdown():
    
    error_angle = searchBall()
    if searching:

	if error_angle != None:
		#if the absolute value of error_angle (distance between the center of the ball and the center of the imagen) is bigger than 15 it will turn till the error is casi 0
		if  abs(error_angle) >= 15:
		    if error_angle > 0:
			angularEntry = 0.3
		    else: 
			angularEntry = -0.3
		# if the ball is in the center it will stop to turn and calculate the perfect way to reach the ball
	    	else:
			angularEntry = 0
			distance = calculateDistance(radiusList)
			pelota = posPelota(x,y,theta,distance)
			anglePelota = atan2((goal.y-pelota.y),(goal.x-pelota.x))
			x1 = 0.2*np.cos(theta)
			y1 = 0.2*np.sin(theta)
			x2 = pelota.x - np.cos(anglePelota)*0.3
			y2 = pelota.y - np.sin(anglePelota)*0.3
			'''print("x1", x1,"y1",y1,"x2",x2,"y2",y2)
			print("x",x,"y",y,pelota)
			print("anglePelota", radians2grades(anglePelota),"theta", radians2grades(theta))'''
			'xRoute = np.linspace(0.2*np.cos(theta),pelota.x - np.cos(anglePelota)*0.2,nPoints)'
			xRoute = np.linspace(x1,x2,num=5)
			yRoute = calculateRoute(pelota,anglePelota,goal,theta)(xRoute)
			route = np.vstack((xRoute, yRoute))
			print("route",route.shape,route)
			searching = False
			moving = True
	#if it cant finde a ball it will turn and search for the ball
        elif angularEntry == 0: 
	    angularEntry = -0.3 
		

	

    if moving == True:	


	#drive to the first point of the route by calculating the differntes between the angles and the position. When both vales are almost cero it will drive to the following point 
	point.x = route[0,idxPoint]
        point.y = route[1,idxPoint]
        inc_x = point.x - x
        inc_y = point.y - y
        angle_to_goal = atan2(inc_y, inc_x)
        errorAngle =  angleError(theta, angle_to_goal)
	print("inc_x", inc_x, "inc_y",inc_y, "a2g", angle_to_goal, "error_angle", errorAngle)
	linearEntry = 0.1
    	if  errorAngle >= 0.25:
            if (angle_to_goal > theta and angle_to_goal - theta < pi) or (angle_to_goal < theta and theta - angle_to_goal > pi):
                angularEntry = 0.3
            else:
                angularEntry = -0.3
	else:
	    angularEntry = 0

	linear_error = sqrt(inc_x**2 + inc_y**2)

	'''print("idxPoint:",idxPoint)
	print("Points:",idxPoint,"%.2f" % route[0,idxPoint],"%.2f" % route[1,idxPoint])
	print("x:","%.2f" % x,"y:","%.2f" % y)
	print("theta:","%.2f" % theta,"angle_to_goal:","%.2f" % angle_to_goal)
	print("errorAngle ","%.2f" % error_angle,"angularEntry ","%.2f" % angularEntry,"linearError ","%.2f" % linear_error, (angle_to_goal > theta and angle_to_goal - theta < pi) or (angle_to_goal < theta and theta - angle_to_goal > pi))'''
	
        
    	if linear_error <= 0.1:
        	idxPoint+=1
		if idxPoint > nPoints - 2:
			angularEntry = 0
			linearEntry = 0
			moving = False
			hitting = True 
			r = 30

    if hitting:
	#when the last point of the routine is reached, the robot will turn and search for the ball. When the ball is in the center of the robot it will speed up (only linear) and hit the ball to play it and make a goal. 
	if error_angle != None:
		if  abs(error_angle) >= 5:
		    if error_angle > 0:
			angularEntry = 0.3
		    else:
			angularEntry = -0.3
	    	else:
			angularEntry = 0
			if(x<=3):
			    linearEntry = 0.3
			else:
			    linearEntry = 0
                        

        elif angularEntry == 0: 
	    angularEntry = 0.5 

	
			

    angularController.setPoint(angularEntry)	
    speed.angular.z = angularController.update(speed.angular.z)

    linearController.setPoint(linearEntry)
    speed.linear.x = linearController.update(speed.linear.x)


    cmd_vel.publish(speed)

    cv2.imshow('RGB',RGB)
    cv2.waitKey(1)
   	    
    thismanager = plt.get_current_fig_manager()
    thismanager.window.wm_geometry("+700+1")
    aline.pop(0)
    aline.append(speed.angular.z)
    bline.pop(0)
    bline.append(angularEntry)
    cline.pop(0)
    cline.append(speed.linear.x)
    dline.pop(0)
    dline.append(linearEntry)
    line1.set_ydata(aline)
    line2.set_ydata(bline)
    line3.set_ydata(cline)
    line4.set_ydata(dline)
    fig.canvas.draw()
    fig.canvas.flush_events()


    	#r.sleep()

        #print(error_angle, speed.angular, speed.linear.x
        #r.sleep()


