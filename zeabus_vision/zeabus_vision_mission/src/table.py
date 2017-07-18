#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
import math
import sys
sys.path.append ('/home/zeabus/catkin_ws/src/src_code/zeabus_vision/zeabus_vision_main/src/')
from vision_lib import *
from sensor_msgs.msg import CompressedImage
# from zeabus_vision_srv_msg.msg import vision_msg_default
# from zeabus_vision_srv_msg.srv import vision_srv_default

img = None
height = None
width = None

def find_table():
    global img, width, height

    lower_bg, upper_bg = get_color('black', 'bottom', 'table')
    while not rospy.is_shutdown():
        while img is None:
            print('None img') 

        image = img.copy()
        imageGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        imageForDraw = img.copy()

        bgr = preprocess_table(image)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        bg = cv2.inRange(hsv, lower_bg, upper_bg)

        cla = clahe(image)
        blur = cv2.bilateralFilter(cla, 15, 75, 75)
        blurClaGray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        blurClaGray = equalization_gray(blurClaGray)

        _, bg = cv2.threshold(bg, 20, 255, cv2.THRESH_BINARY_INV)
        # bg = np.invert(bg)
        bg = open_morph(bg, get_kernal())
        bg = close(bg, get_kernal())

        ret, thresh = cv2.threshold(imageGray, 60, 255, cv2.THRESH_BINARY_INV)
        _, thresh1 = cv2.threshold(blurClaGray, 50, 255, cv2.THRESH_BINARY_INV)

        # thresh1 = cv2.subtract(thresh1, bg)

        thresh1 = close(thresh1, get_kernal())

        _, contours, hierarchy = cv2.findContours(thresh1.copy(), 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)

        _, rectContours, _ = cv2.findContours(bg.copy(),
                                            cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)


        # cv2.drawContours(imageForDraw, contours, -1, (0, 0, 255,), 2) 
        for c in contours:
            M = cv2.moments(c)
            if len(c) >= 5: 
                ellipse = (x,y), (mn,mj), angle = cv2.fitEllipse(c)
                if mj < 20 or mj > 100:
                    continue
                if mn < 20:
                    continue
                # print('ellipse', ellipse)
                # cv2.ellipse(imageForDraw, ellipse, (255,255,0),2)

        for c in rectContours:
            M = cv2.moments(c)
            rect = (x,y), (ww, hh), _ = cv2.minAreaRect(c)
            realArea = cv2.contourArea(c)
            angle = 90-Oreintation(M)[0]*180/math.pi 
            if realArea < 10000:
                continue
            print('realArea',realArea)
            print('angle',angle)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(imageForDraw,[box],0,(0,0,255),2)

        cv2.imshow('bg', bg)
        # cv2.imshow('thresh1', thresh1)
        # cv2.imshow('thresh', thresh)
        # cv2.imshow('grayScale', imageGray)
        cv2.imshow('imageForDraw', imageForDraw)
        cv2.waitKey(30)

def img_callback(msg):
    global img, width, height
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (640, 384))

    height, width, _ = img.shape
    

if __name__ == '__main__':
    rospy.init_node('findBin')
    bag = '/leftcam_bottom/image_raw/compressed'
    topic = '/bottom/left/image_raw/compressed'
    bot = '/bottom/left/image_raw/compressed'
    rospy.Subscriber(bot, CompressedImage, img_callback)
    find_table()
