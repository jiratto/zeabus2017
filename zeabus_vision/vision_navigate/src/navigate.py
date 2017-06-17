#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
import math
from sensor_msgs.msg import CompressedImage

img = None

lower_yellow = np.array([ 87, 26, 221])
upper_yellow = np.array([ 96, 161, 255])


def navigate():
    global img
    im = None
    kernel_1 = np.ones((11,11), np.uint8)
    while not rospy.is_shutdown():
        while img is None:
            print("img : None")
        im = img.copy()
        hsv = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
        im_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        dilation = cv2.dilate(im_yellow, kernel_1, iterations =  1)
        _, contours, hierarchy = cv2.findContours(dilation.copy(), 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            M = cv2.moments(c)
            rect = (x,y), (ww,hh), angle = cv2.minAreaRect(c)
            area = ww*hh

            # epsilon = 0.1*cv2.arcLength(c, t)
            # approx = cv2.approxPolyDP(c, epsilon,t)
            
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            print('draw')
            cv2.drawContours(dilation, [box], 0, (0, 0, 255,), 2) 


        cv2.imshow('yellow?',im_yellow)
        cv2.imshow('img', img)
        cv2.imshow('dilate', dilation)
        cv2.waitKey(30)


def img_callback(msg):
    global img

    arr = np.fromstring( msg.data, np.uint8)
    img = cv2.imdecode(arr, 1)

if __name__ == '__main__':
    rospy.init_node('Navigate')
    bot = '/rightcam_bottom/image_raw/compressed'
    top = '/rightcam_top/image_raw/compressed'
    rospy.Subscriber(top, CompressedImage, img_callback)
    navigate()