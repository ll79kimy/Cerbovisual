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
from controller import PID

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


rospy.init_node("ballFollower")
r = rospy.Rate(10) #10hz
rospy.Subscriber("/camera/rgb/image_color", Image, getRGB)
rospy.Subscriber("/odom", Odometry, getOdom)

cmd_vel = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size = 1)


bridge = CvBridge()

speed = Twist()
speed.angular.z = 0
speed.linear.x = 0


angularController = PID()
angularController.setKp(0.001)
angularController.setKi(0.05)
angularController.setKd(0.005)

entry = 0

plt.ion()

xline = range(100)
aline = [0]*100
bline = [0]*100
cline = [0]*100
dline = [0]*100

fig = plt.figure()

ax1 = fig.add_subplot(221)
ax1.set_ylim(-0.3, 0.3)
line1, = ax1.plot(xline, aline, 'r-')

ax2 = fig.add_subplot(222)
ax2.set_ylim(-0.3, 0.3)
line2, = ax2.plot(xline, bline, 'b-')

ax3 = fig.add_subplot(223)
ax3.set_ylim(-0.3, 0.3)
line3, = ax3.plot(xline, cline, 'g-')

ax4 = fig.add_subplot(224)
ax4.set_ylim(-0.3, 0.3)
line4, = ax4.plot(xline, dline, 'y-')


while not rospy.is_shutdown():
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
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(RGB, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                cv2.circle(RGB, center, 5, (0, 0, 255), -1)

            colorMask = cv2.cvtColor(colorMask, cv2.COLOR_GRAY2RGB)
            error_angle = RGB.shape[1]/2-center[0]

            if  abs(error_angle) >= 10:
                if error_angle > 0:
                    entry = 0.3
                else:
                    entry = -0.3
            else:
                    entry = 0

            angularController.setPoint(entry)
            P, I, D, angularValue = angularController.update(angularValue)
            speed.angular.z = angularValue*1
            #print(P,I,D,angularController.getError
            print(angularController.getIntegrator(), angularController.getDerivator())
            cmd_vel.publish(speed)

        cv2.imshow('RGB',RGB)
        cv2.waitKey(1)
        thismanager = plt.get_current_fig_manager()
        thismanager.window.wm_geometry("+800+1")
        aline.pop(0)
        aline.append(speed.angular.z)
        bline.pop(0)
        bline.append(P)
        cline.pop(0)
        cline.append(I)
        dline.pop(0)
        dline.append(D)
        line1.set_ydata(aline)
        line2.set_ydata(bline)
        line3.set_ydata(cline)
        line4.set_ydata(dline)
        fig.canvas.draw()
        fig.canvas.flush_events()



        #print(error_angle, speed.angular, speed.linear.x
        #r.sleep()
