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

lw2 = np.array([ 66, 32, 177])
up2 = np.array([ 92, 134, 255])

lw3 = np.array([ 13, 15, 67])
up3 = np.array([ 97, 97, 214])


def navigate():
    global img
    im = None
    kernel_1 = np.ones((9,9), np.uint8)
    kernel_2 = np.ones((17,17), np.uint8)
    while not rospy.is_shutdown():
        while img is None:
            print("img : None")
        im = img.copy()
        hsv = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
        im_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        im_yellow2 = cv2.inRange(hsv, lw2, up2)
        im_yellow += im_yellow2
        delete = cv2.inRange(hsv, lw3, up3)
        im_yellow -= delete
        dilation = cv2.dilate(im_yellow, kernel_1, iterations =  1)
        closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel_2)
        _, contours, hierarchy = cv2.findContours(dilation.copy(), 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)
        max = 0
        for c in contours:
            M = cv2.moments(c)
            rect = (x,y), (ww,hh), angle = cv2.minAreaRect(c)
            area = ww*hh
            if area < 17000 or area > 150000:
                continue
            print(area)
            if max<area :
                max = area
            # epsilon = 0.1*cv2.arcLength(c, t)
            # approx = cv2.approxPolyDP(c, epsilon,t)
            
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            print('draw')
            cv2.drawContours(img, [box], -1, (0, 0, 255,), 2) 
            cv2.drawContours(img, c, -1, (0 , 255, 0), 2)
            cv2.circle(img ,(int(x), int(y)), 5, (0, 0, 255), -1)
        
        print('max: ', max)
        cv2.imshow('yellow?',im_yellow)
        cv2.imshow('img', img)
        cv2.imshow('closing', closing)
        # cv2.imshow('dilate', dilation)
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