#!/usr/bin/env python

import numpy as np
import rospy
#import picamera
from cv_bridge import CvBridge
bridge = CvBridge()
from sensor_msgs.msg import Image, CompressedImage
from ggcnn_humanseg_ros.msg import GraspPrediction
from darknet_ros_msgs.msg import BoundingBoxes
import cv2
import time
import message_filters


class Collect(object):
    def __init__(self):
        self.interface_topic = rospy.get_param('/ggcnn_humanseg/interface/topic')
        self.darknet_topic = rospy.get_param('/ggcnn_humanseg/subscription/darknet')

        self.image_sub = message_filters.Subscriber(self.interface_topic, GraspPrediction)
		self.grasp_point_sub = message_filters.Subscriber(self.darknet_topic, BoundingBoxes)
		self.ts = message_filters.TimeSynchronizer([image_sub, grasp_point_sub], 1)
		self.ts.registerCallback(self.callback)

        self.pub_ = rospy.Publisher('/DC', Image, queue_size=1)

    def callback(self, data1, data2):
        self.IMG = bridge.imgmsg_to_cv2(data1, desired_encoding = "bgr8")

        x, y, z = data2.pose.position.x, data2.pose.position.y, data2.pose.position.z
        self.IMG = cv2.circle(self.IMG, (x,y), radius=1, color=(255, 255, 0), thickness=-1)


        self.pub_.publish(bridge.cv2_to_imgmsg(self.IMG1))


if __name__ == "__main__":
    rospy.init_node("collect")
    rospy.loginfo('ss')
    collecter = Collect()
    rospy.spin()
