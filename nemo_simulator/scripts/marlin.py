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
        velocity.angular.z = 1
        while not rospy.is_shutdown():
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
        rospy.loginfo(f"Sonar info: {msg.point.x} {msg.point.y} {msg.point.z}")
        self.transform(self.odomOrientation, self.nemoPos)
        rospy.loginfo(f"Transformed sonar info: {self.nemoRealPos.x} {self.nemoRealPos.y} {self.nemoRealPos.z}")
    
    def receiveOdom(self, msg):
        self.odomOrientation = msg.pose.pose.orientation


if __name__ == '__main__':
    marlin = Marlin()
    marlin.swim()

