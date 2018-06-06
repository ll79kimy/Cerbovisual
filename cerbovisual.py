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

def promediarLista(lista):
    #sum=0.0
    #for i in range(0,len(lista)):
        #sum=sum+lista[i]

    #return sum/len(lista)
   return float(sum(lista)) / max(len(lista), 1)

def newOdom(msg):
	global x
	global y
	global theta

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	rot_q = msg.pose.pose.orientation
	(r, p, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	return x,y,theta

def posPelota (x,y,theta, distance):
	pelota.x = x+math.cos(theta)*distance 
	pelota.y = y+math.sin(theta)*distance
	return pelota

def calculateDistance(radiusList):
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
	print ("Ai", Ai)
	print ("v", v)
	m = Ai*v
	print ("m", m,"m1", m1,"m2", m2,"b2", b2, "x1", x1,"y1", y1,"x2", x2,"y2", y2)
	#print("Ys",y[0][0],y[1][0],y[2][0],y[3][0])
	
	def cuadratic(x):	
		#return a*x**2+b*x+c

		return m[0]*x**3+m[1]*x**2+m[2]*x+m[3]
	return cuadratic

def calculateRouteLin(pelota, anglePelota):
	x2 = pelota.x - np.cos(anglePelota)*0.2
	y2 = pelota.y - np.sin(anglePelota)*0.2
	theta2 = atan2(y2, x2)
	m1 = y2/x2
	def cuadratic(x):	
		#return a*x**2+b*x+c
		return m1*x
	return cuadratic

def angleError(a, b):
    diff = abs(a - b);
    return diff

def searchBall():
    global i
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
            if radius > 15:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(RGB, (int(rx), int(ry)), int(radius),(0, 255, 255), 2)
                cv2.circle(RGB, center, 5, (0, 0, 255), -1)	
		colorMask = cv2.cvtColor(colorMask, cv2.COLOR_GRAY2RGB)
		return RGB.shape[1]/2-center[0]
    return None 

rospy.init_node("ballFollower")
r = rospy.Rate(10) #10hz
rospy.Subscriber("/camera/rgb/image_color", Image, getRGB)
rospy.Subscriber("/odom", Odometry, getOdom)

cmd_vel = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size = 1)
reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)


bridge = CvBridge()

speed = Twist()
speed.angular.z = 0
speed.linear.x = 0


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

i = 0
radiusList = [0]*20
distance = 1

pelota = Point()
anglePelota = 0

goal = Point()
goal.x = 4
goal.y = -1

point = Point()

searching = True
moving = False
hitting = False
route = None

idxPoint = 0
nPoints = 5

reset_odom.publish(Empty())
while not rospy.is_shutdown():
    
    error_angle = searchBall()
    if searching:

	if error_angle != None:
		if  abs(error_angle) >= 15:
		    if error_angle > 0:
			angularEntry = 0.3
		    else:
			angularEntry = -0.3
	    	else:
			angularEntry = 0
			distance = calculateDistance(radiusList)
			pelota = posPelota(x,y,theta,distance)
			anglePelota = atan2((goal.y-pelota.y),(goal.x-pelota.x))
			x1 = 0.2*np.cos(theta)
			y1 = 0.2*np.sin(theta)
			x2 = pelota.x - np.cos(anglePelota)*0.2
			y2 = pelota.y - np.sin(anglePelota)*0.2
			'''print("x1", x1,"y1",y1,"x2",x2,"y2",y2)
			print("x",x,"y",y,pelota)
			print("anglePelota", radians2grades(anglePelota),"theta", radians2grades(theta))'''
			xRoute = np.linspace(0.2*np.cos(theta),pelota.x - np.cos(anglePelota)*0.2,nPoints)
			xRoute = np.linspace(x1,x2,num=5)
			yRoute = calculateRoute(pelota,anglePelota,goal,theta)(xRoute)
			route = np.vstack((xRoute, yRoute))
			print("route",route.shape,route)
			searching = False
			moving = True
        elif angularEntry == 0: 
	    angularEntry = -0.3 
		

	

    if moving == True:	
	#linear
	'''x2 = pelota.x - np.cos(anglePelota)*0.2
	y2 = pelota.y - np.sin(anglePelota)*0.2
        inc_x = x2 - x
        inc_y = y2 - y
        angle_to_goal = atan2(inc_y, inc_x)
        error_angle =  angleError(theta, angle_to_goal)
	print("inc_x", inc_x, "inc_y",inc_y, "a2g", angle_to_goal, "error_angle", error_angle)
	#linearEntry = 0.1
	while abs(error_angle) > 0.07:
    		if  error_angle >= 0.1:
        	    if (angle_to_goal > theta and angle_to_goal - theta < pi) or (angle_to_goal < theta and theta - angle_to_goal > pi):
        	        angularEntry = 0.3
        	    else:
        	        angularEntry = -0.3
		linearEntry = 0
		angularController.setPoint(angularEntry)	
		speed.angular.z = angularController.update(speed.angular.z)

		linearController.setPoint(linearEntry)
		speed.linear.x = linearController.update(speed.linear.x)

		cmd_vel.publish(speed)

	linear_error = sqrt(inc_x**2 + inc_y**2)
	while abs(linear_error) > 0.05:
		linearEntry = 0.1		
		linearController.setPoint(linearEntry)
		angularEntry = 0
		linearEntry = 0
		angularController.setPoint(angularEntry)	
		speed.angular.z = angularController.update(speed.angular.z)

		linearController.setPoint(linearEntry)
		speed.linear.x = linearController.update(speed.linear.x)

		cmd_vel.publish(speed)

        inc_x = goal.x - x
        inc_y = goal.y - y	
        angle_to_goal = atan2(inc_y, inc_x)
        error_angle =  angleError(theta, angle_to_goal) 
	while abs(error_angle) > 0.07:
    		if  error_angle >= 0.1:
        	    if (angle_to_goal > theta and angle_to_goal - theta < pi) or (angle_to_goal < theta and theta - angle_to_goal > pi):
        	        angularEntry = 0.3
        	    else:
        	        angularEntry = -0.3
		linearEntry = 0
		angularController.setPoint(angularEntry)	
		speed.angular.z = angularController.update(speed.angular.z)

		linearController.setPoint(linearEntry)
		speed.linear.x = linearController.update(speed.linear.x)

		cmd_vel.publish(speed)'''

	

	#normal
	point.x = route[0,idxPoint]
        point.y = route[1,idxPoint]
        inc_x = point.x - x
        inc_y = point.y - y
        angle_to_goal = atan2(inc_y, inc_x)
        error_angle =  angleError(theta, angle_to_goal)
	print("inc_x", inc_x, "inc_y",inc_y, "a2g", angle_to_goal, "error_angle", error_angle)
	linearEntry = 0.1
    	if  error_angle >= 0.25:
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

