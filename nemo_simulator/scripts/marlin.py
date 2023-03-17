#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist

class Marlin:
    def __init__(self):
        rospy.init_node('marlin', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/sonar_data', PointStamped, self.receiveSonar)
        nemoPos = PointStamped()

    def swim(self):
        rate = rospy.Rate(5)
        velocity = Twist()
        # teste = PointStamped()
        # num = teste.point.x
        num = nemoPos.point.x.data
        velocity.linear.x = 1
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
        
        nemoPos = msg
        rospy.loginfo(f"Sonar info: {nemoPos.point.x} {msg.point.y} {msg.point.z}")

if __name__ == '__main__':
    marlin = Marlin()
    marlin.swim()