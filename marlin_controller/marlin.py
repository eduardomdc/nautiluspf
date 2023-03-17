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

    def swim(self):
        rate = rospy.Rate(5)
        velocity = Twist()
        
        while not rospy.is_shutdown():
            rospy.loginfo(f"Transformed sonar info: {self.nemoRealPos.x} {self.nemoRealPos.y} {self.nemoRealPos.z}")
            self.steering(velocity)
            self.pub.publish(velocity)
            rate.sleep()

    def steering(self, velocity):
        #foward motion
        velocity.linear.y = 0.5
        #control change of direction
        if (self.nemoRealPos.y > 0):
            if (self.nemoRealPos.x < 0):
                velocity.angular.z = -2
            else:
                velocity.angular.z = 2
        else:
            velocity.angular.z = 4

        #if position is the same, reverse

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
        self.transform(self.odomOrientation, self.nemoPos)
    
    def receiveOdom(self, msg):
        self.odomOrientation = msg.pose.pose.orientation
        self.odomPosition = msg.pose.pose.position


if __name__ == '__main__':
    marlin = Marlin()
    marlin.swim()