#!/usr/bin/env python3
import cv2
import sys
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node("cv_bridge", anonymous=True)

class CameraHandler:
    def __init__(self):
        self.img = np.zeros((1000, 1000))
        self.bridge = CvBridge()
        rospy.Subscriber("camera/image_raw", Image, self.callback)
        rospy.wait_for_message("camera/image_raw", Image)


    def callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        except CvBridgeError, e:
            print("Frame Dropped: ", e)


    def fetchImg(self):
        return self.img


    def showImg(self):
        cv2.imshow("Frontal Camera", self.img)



handler = CameraHandler()

#EXAMPLE
# while not rospy.is_shutdown():
#     print(handler.fetchImg().shape)
#     feed = handler.fetchImg()
    
#     cv2.imshow("feed", feed)
#     cv2.waitKey()

