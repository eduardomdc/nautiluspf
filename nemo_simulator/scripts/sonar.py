#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
import numpy as np
from geometry_msgs.msg import PointStamped


rospy.init_node("sonar", anonymous=True)

class FishTracker():
    def __init__(self, model_name):
        self.model_name = model_name
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        rospy.wait_for_message("/gazebo/model_states", ModelStates)


    def callback(self, msg):
        index = msg.name.index(self.model_name)
        pos = msg.pose[index].position
        
        self.pos = np.array((pos.x, pos.y, pos.z))



nemo = FishTracker("nemo")
marlin = FishTracker("marlin")
rate = rospy.Rate(rospy.get_param("rate"))
pub = rospy.Publisher("sonar_data", PointStamped)

def genMsg(delta):
    error = rospy.get_param("noise-mag")
    noise = np.random.normal(-error, error, size=3)

    msg = PointStamped()
    msg.header.stamp = rospy.Time.now()

    msg.point.x = delta[0] + noise[0]
    msg.point.y = delta[1] + noise[1]
    msg.point.z = delta[2] + noise[2]

    return msg

while not rospy.is_shutdown():
    delta = ( marlin.pos - nemo.pos )
    msg = genMsg(delta)
    pub.publish(msg)
    rate.sleep()

rospy.spin()



