#!/usr/bin/env python
import sys
import cv2
import time
import numpy as np
from scipy.ndimage import filters
import rospy
from sensor_msgs.msg import CompressedImage
from adjust_exposure import *


class ImagePreprocessing:

    def __init__(self):
        self.image = None
        self.state = 0
        self.subTopicLeft = "/leftcam_top/image_raw/compressed"
        self.pubTopic = "/processing/image_raw/compressed"

        self.pub = rospy.Publisher(
            self.pubTopic, CompressedImage, queue_size=1)
        self.subLeft = rospy.Subscriber(
            self.subTopicLeft, CompressedImage, self.callback,  queue_size=1)

    def callback(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        self.image = cv2.imdecode(np_arr, 1)

    def pre_processing(self):
        while(self.image is None):
            rospy.sleep(0.01)

        msg = CompressedImage()
        msg.format = "jpeg"

        while not rospy.is_shutdown():
            self.state += 1

            print(self.state)
            if self.state % 3 == 0:
                print(self.state)
                msg.header.stamp = rospy.Time.now()
                msg.data = np.array(cv2.imencode(
                    '.jpg', self.image)[1]).tostring()
                self.pub.publish(msg)
                self.state = 0

if __name__ == '__main__':
    rospy.init_node('image_preprocessing', anonymous=True)
    preImg = ImagePreprocessing()
    preImg.pre_processing()
