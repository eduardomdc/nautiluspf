#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

class Marlin:
    def __init__(self):
        rospy.init_node('marlin', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/sonar_data', PointStamped, self.receiveSonar)
        rospy.Subscriber('/odom', Odometry, self.receiveOdom)
        nemoPos = Point()
        odomOrientation = Quaternion()

    def swim(self):
        rate = rospy.Rate(5)
        velocity = Twist()
        # teste = PointStamped()
        # num = teste.point.x
        #num = self.nemoPos.point.x #nemoPos is undefinel for me
        velocity.linear.y = 0.5 
        while not rospy.is_shutdown():
            self.pub.publish(velocity)
            rate.sleep()
    
    def receiveSonar(self, msg):
        # PointStamped:
        #   Header:
        #        uint32 seq
        #        time stamp
        #        string frame_id
        #   Point:
        #        x, y, z
        
        nemoPos = msg.point
        rospy.loginfo(f"Sonar info: {nemoPos.x} {msg.point.y} {msg.point.z}")
    
    def receiveOdom(self, msg):
        odomOrientation = msg.pose.pose.orientation


if __name__ == '__main__':
    marlin = Marlin()
    marlin.swim()

