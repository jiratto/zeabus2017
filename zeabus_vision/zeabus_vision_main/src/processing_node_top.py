#!/usr/bin/env python
import sys
import cv2
import time
import numpy as np
from scipy.ndimage import filters
import rospy
from sensor_msgs.msg import CompressedImage
from vision_lib import *


class ImagePreprocessing:

    def __init__(self):
        self.imageL = None
        self.imageR = None
        self.imageC = None
        self.width = int(1152 / 3)
        self.height = int(870 / 3)
        self.state = 0
        self.start = False
        self.subTopicL = "/stereo/left/image_raw/compressed"
        self.subTopicR = "/stereo/right/image_raw/compressed"
        self.subTopicC = "/top/center/image_rect_color/compressed"
        # self.subTopicC = "/bottom/left/image_raw/compressed"
        self.pubTopic = "/processing"
        self.nodeL = 'ueye_cam_nodelet_left/'
        self.nodeR = 'ueye_cam_nodelet_right/'

        self.pub = rospy.Publisher(
            self.pubTopic, CompressedImage, queue_size=10)
        self.subLeft = rospy.Subscriber(
            self.subTopicL, CompressedImage, self.left_callback,  queue_size=10)
        self.subRight = rospy.Subscriber(
            self.subTopicR, CompressedImage, self.right_callback,  queue_size=10)
        self.subCenter = rospy.Subscriber(
            self.subTopicC, CompressedImage, self.center_callback,  queue_size=10)
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

    def center_callback(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        self.imageC = cv2.resize(cv2.imdecode(
            np_arr, 1), (self.width, self.height))

    def set_param(self, client, param, value):
        params = {str(param): value}
        config = client.update_configuration(params)

    def pre_processing(self):
        # fgbg = cv2.bgsegm.createBackgroundSubtractorMOG()
        msg = CompressedImage()
        msg.format = "jpeg"
        while not rospy.is_shutdown():
            # print('while process')
            if self.imageC is None:
                print('Image None in loop')
                continue
            hsv = cv2.cvtColor(self.imageC, cv2.COLOR_BGR2HSV)
            h, s, v = cv2.split(hsv)
            vMean = cv2.mean(v)[0]
            vBright = int((255 - vMean + 5) / 2)
            vDark = int((vMean - 5) / 2)

            print (vBright, vDark)
            imageBlur = cv2.bilateralFilter(self.imageC, 9, 75, 75)
            imageCLAHE = clahe(self.imageC)
            imageDark = brightness(imageCLAHE, -vDark)
            imageBright = brightness(imageCLAHE, vBright)
            imageEqu = equalization_bgr(self.imageC)
            imageDark1 = brightness(imageEqu, -vDark)
            imageBright1 = brightness(imageEqu, vBright)

            images = [imageDark, imageBright, self.imageC,
                      imageCLAHE, imageEqu, imageDark1, imageBright1]

            # images = [imageDark, imageBright, self.imageC,
            #           imageCLAHE, imageEqu]
            merge_mertens = cv2.createMergeMertens()
            res_mertens = merge_mertens.process(images)
            res_mertens = np.clip(res_mertens * 255, 0, 255).astype('uint8')

            gray = cv2.cvtColor(res_mertens, cv2.COLOR_BGR2GRAY)

            # fgmask = fgbg.apply(imageEqu)

            cv2.imshow('gray', gray)
            cv2.imshow('CLAHE', imageCLAHE)
            # cv2.imshow('equ', imageEqu)
            cv2.imshow('mertens', res_mertens)
            cv2.imshow('dark', imageDark)
            cv2.imshow('bright', imageBright)
            # cv2.imshow('dark1', imageDark1)
            # cv2.imshow('bright1', imageBright1)
            cv2.imshow('imageC', self.imageC)
            msg.header.stamp = rospy.Time.now()
            msg.data = np.array(cv2.imencode(
                '.jpg', res_mertens)[1]).tostring()
            self.pub.publish(msg)
            # publish_result(res_mertens, 'bgr', self.pubTopic)
            rospy.sleep(0.1)
            key = cv2.waitKey(1) & 0xff
            if key == ord('q'):
                break
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('image_preprocessing', anonymous=True)
    preImg = ImagePreprocessing()
    preImg.pre_processing()
