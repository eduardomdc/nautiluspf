#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import numpy as np

import cv2
import numpy as np
from collections import deque

#for keeping center points of object
buffer_size = 16
pts = deque(maxlen=buffer_size)

#capture
cap = cv2.VideoCapture('/camera_link')
cap.set(3,960)
cap.set(4,480)

# green HSV
greenLower = (40, 40, 40)
greenUpper = (70, 255, 255)

while True:

    success, imgOriginal = cap.read()

    if success:
        
        #blur
        blurred = cv2.GaussianBlur(imgOriginal, (11,11), 0)

        #HSV
        hsv = cv2.ctvColor(blurred, cv2.COLOR_BGR2HSV)
        cv2.imshow("HSV IMAGE", hsv)

        #mask for green
        mask = cv2.inRange(hsv, greenLower, greenUpper)

        #deleting noises which are in area of mark
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cv2.imshow("Mask + Erosion + Dilation", mask)

        #countours
        contours,_= cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,    cv2.CHAIN_APPROX_SIMPLE)
        center = None

        if len(contours) > 0:

            #get max contour
            c = max(contours, key=cv2.contourArea)

            #return rectangle
            rect = cv2.minAreaRect(c)
            ((x,y), rotation) = rect

            s = f"x: {np.round(x)}, y: {np.round(y)}, rotation: {np.round(rotation)}"
            print(s)

            #box
            box = cv2.boxPoints(rect)
            box = np.int64(box)

            #moment
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            #draw contour
            cv2.drawContours(imgOriginal, [box], 0, (0, 255, 255), 2)

            #point in center
            cv2.circle(imgOriginal, center, 5, (255, 0, 255), -1)

            # deque
            pts.appendleft(center)
            for i in range(1, len(pts)):

                if pts[i - 1] is None or pts[i] is None: continue

                cv2.line(imgOriginal, pts[i - 1], pts[i], (0, 255, 0), 3)

        cv2.imshow("DETECTED IMAGE", imgOriginal)



    if cv2.waitKey(1) & 0xFF == ord("q"): break





class Marlin:
    def __init__(self):
        rospy.init_node('marlin', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/sonar_data', PointStamped, self.receiveSonar)
        rospy.Subscriber('/odom', Odometry, self.receiveOdom)
        self.nemoPos = Point()
        self.nemoRealPos = Point()
        self.odomOrientation = Quaternion()

    def swim(self):
        rate = rospy.Rate(5)
        velocity = Twist()
        # teste = PointStamped()
        # num = teste.point.x
        #num = self.nemoPos.point.x #nemoPos is undefinel for me
        while not rospy.is_shutdown():
            rospy.loginfo(f"Transformed sonar info: {self.nemoRealPos.x} {self.nemoRealPos.y} {self.nemoRealPos.z}")
            if (self.nemoRealPos.y == 0):
                velocity.angular.z = 1
            self.pub.publish(velocity)
            rate.sleep()

    def transform(self, orientation, absPos):
        # we need to rotate the nemoPos to account for the orientation
        # of the robot around the Z axis
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        #rotation matrix for Z-axis rotation
        rotationMatrix = np.array([[math.cos(yaw), math.sin(yaw)],
                                   [-math.sin(yaw), math.cos(yaw)]])
        (self.nemoRealPos.x, self.nemoRealPos.y) = np.dot(rotationMatrix, [self.nemoPos.x, self.nemoPos.y])
        self.nemoRealPos.y = -self.nemoRealPos.y # +y is in front

    def receiveSonar(self, msg):
        self.nemoPos = msg.point
        #rospy.loginfo(f"Sonar info: {msg.point.x} {msg.point.y} {msg.point.z}")
        self.transform(self.odomOrientation, self.nemoPos)
        #rospy.loginfo(f"Transformed sonar info: {self.nemoRealPos.x} {self.nemoRealPos.y} {self.nemoRealPos.z}")
    
    def receiveOdom(self, msg):
        self.odomOrientation = msg.pose.pose.orientation


if __name__ == '__main__':
    marlin = Marlin()
    marlin.swim()





