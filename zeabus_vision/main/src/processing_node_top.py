#!/usr/bin/env python
import sys
import cv2
import time
import numpy as np
from scipy.ndimage import filters
import rospy
from sensor_msgs.msg import CompressedImage


class ImagePreprocessing:

    def __init__(self):
        self.imageL = None
        self.imageR = None
        self.width = int(1936 / 3)
        self.height = int(1216 / 3)
        self.state = 0
        self.start = False
        self.subTopicL = "/stereo/left/image_raw/compressed"
        self.subTopicR = "/stereo/right/image_raw/compressed"
        self.pubTopic = "/processing/image_raw/compressed"
        self.nodeL = 'ueye_cam_nodelet_left/'
        self.nodeR = 'ueye_cam_nodelet_right/'

        self.pub = rospy.Publisher(
            self.pubTopic, CompressedImage, queue_size=10)
        self.subLeft = rospy.Subscriber(
            self.subTopicL, CompressedImage, self.left_callback,  queue_size=10)
        self.subRight = rospy.Subscriber(
            self.subTopicR, CompressedImage, self.right_callback,  queue_size=10)

        # self.clientL = dynamic_reconfigure.client.Client(self.nodeL)
        # self.clientR = dynamic_reconfigure.client.Client(self.nodeR)

    def left_callback(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        self.imageL = cv2.resize(cv2.imdecode(
            np_arr, 1), (self.width, self.height))

    def right_callback(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        self.imageR = cv2.resize(cv2.imdecode(
            np_arr, 1), (self.width, self.height))

    def set_param(self, client, param, value):
        params = {str(param): value}
        config = client.update_configuration(params)

    def pre_processing(self):
        while self.imageL is None or self.imageR is None:
            print('Image None1')

        msg = CompressedImage()
        msg.format = "jpeg"

        # images = []
        while not rospy.is_shutdown():
            print('while process')
            if self.imageL is None or self.imageR is None:
                print('Image None2')
                continue

            images = [self.imageL, self.imageR]
            merge_mertens = cv2.createMergeMertens()
            res_mertens = merge_mertens.process(images)
            cv2.imshow('mertens', res_mertens)

            res = res_mertens

            self.start = False

            msg.header.stamp = rospy.Time.now()
            msg.data = np.array(cv2.imencode('.jpg', res)[1]).tostring()
            self.pub.publish(msg)

            key = cv2.waitKey(1) & 0xff
            if key == ord('q'):
                break
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('image_preprocessing', anonymous=True)
    preImg = ImagePreprocessing()
    preImg.pre_processing()
