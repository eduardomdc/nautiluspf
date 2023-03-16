#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

class Marlin:
    def __init__(self):
        rospy.init_node('marlin', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def swim(self):    
        rate = rospy.Rate(5)
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 1
        msg.linear.z = 0
        msg.angular.z = 1

        while not rospy.is_shutdown():
            self.pub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    marlin = Marlin()
    marlin.swim()