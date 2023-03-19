#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped, Point, Twist, Vector3, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import numpy as np

import cv2
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

def isSamePlace(point, lastPoint):
    #calculates the distance between points and
    #returns true if the distance is bellow a threshold
    if distance(point, lastPoint)<0.1:
        return True
    else:
        return False

def distance(a, b = None):
    if (a != None and b == None):
        return (a.x)**2+(a.y)**2
    else:
        return (a.x-b.x)**2 + (a.y-b.y)**2

class Marlin:
    def __init__(self):
        rospy.init_node('marlin', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/sonar_data', PointStamped, self.receiveSonar)
        rospy.Subscriber('/odom', Odometry, self.receiveOdom)
        self.nemoPos = Point()
        self.nemoRealPos = Point()
        self.odomOrientation = Quaternion()
        self.odomPosition = Point()
        self.odomLastPosition = Point() #marlin's position on last sonar scan
        self.stuck = False #boolean that indicates if stuck procedure should be executed

    def swim(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            velocity = Twist()
            rospy.loginfo(f"Transformed sonar info: {self.nemoRealPos.x} {self.nemoRealPos.y} {self.nemoRealPos.z}")
            if not self.stuck:
                self.steering(velocity)
            else:
                self.unstuck(velocity)
            self.pub.publish(velocity)
            rate.sleep()

    def steering(self, velocity):
        # if distance(self.nemoRealPos) > 100:
        #     fastMode()
        # else:
        speed = min(10, distance(self.nemoRealPos)/10)
        velocity.linear.y = speed
        #velocity.linear.y = 2
        #control change of direction
        if self.nemoRealPos.y > 0:
            if (self.nemoRealPos.x < 0):
                velocity.angular.z = -1
            else:
                velocity.angular.z = 1
        else:
            # reverse if it's behind you
            velocity.linear.y = -0.5
            velocity.angular.z = 4

    def unstuck(self, velocity):
        #big swipe to go around blocker
        if self.nemoRealPos.x < 0:
            velocity.angular.z = 6
        else:
            velocity.angular.z = -6
        
        if self.nemoRealPos.y > 0:
            #if marlin is facing nemo then reverse
            velocity.linear.y = -10
        else:
            #otherwise go forward
            velocity.linear.y = 10
            velocity.angular.z *= -1


    def transform(self, orientation, absPos):
        # we need to rotate the nemoPos to account for the orientation
        # of the robot around the Z axis
        orientation_vec = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_vec)
        #rotation matrix for Z-axis rotation
        rotationMatrix = np.array([[math.cos(yaw), math.sin(yaw)],
                                   [-math.sin(yaw), math.cos(yaw)]])
        (self.nemoRealPos.x, self.nemoRealPos.y) = np.dot(rotationMatrix, [self.nemoPos.x, self.nemoPos.y])
        self.nemoRealPos.y = -self.nemoRealPos.y # +y is in front

    def receiveSonar(self, msg):
        self.nemoPos = msg.point
        self.transform(self.odomOrientation, self.nemoPos)
        if (isSamePlace(self.odomPosition, self.odomLastPosition) 
            and distance(self.nemoRealPos) > 2):
            self.stuck = True
        else:
            self.stuck = False
        self.odomLastPosition = self.odomPosition
    
    def receiveOdom(self, msg):
        self.odomOrientation = msg.pose.pose.orientation
        self.odomPosition = msg.pose.pose.position


if __name__ == '__main__':
    marlin = Marlin()
    marlin.swim()