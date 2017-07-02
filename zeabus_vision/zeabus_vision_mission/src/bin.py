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
from zeabus_vision_srv_msg.msg import vision_msg_default
from zeabus_vision_srv_msg.srv import vision_srv_default

img = None
height = None
width = None

def adjust_gamma(image, gamma=1.0):
    invGamma = 1.0/gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
    for i in np.arange(0, 256)]).astype("uint8")

    return cv2.LUT(image, table)

def canny(image, threshold1, threshold2):
    return cv2.Canny(image, threshold1, threshold2)

<<<<<<< HEAD
def find_bin():
=======
def find_bin(msg):
>>>>>>> bd55591b68580b349afad1141eda1399efbbdd74
    global img

    lowerOrange, upperOrange = getColor('orange', 'down')
    lowerWhite, upperWhite = getColor('white', 'down')
<<<<<<< HEAD

=======
    res = vision_msg_navigate()
>>>>>>> bd55591b68580b349afad1141eda1399efbbdd74
    while not rospy.is_shutdown():
        while img is None:
            print('None')
        image = img.copy()
<<<<<<< HEAD
        edges = cv2.Canny(image, 100, 200)
        imageForDraw = img.copy()
        hsv = equalization(image)
        hsv = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        # median = cv2.medianBlur(hsv,5)
        gray = cv2.cvtColor(hsv, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,180,255,cv2.THRESH_BINARY)
        # orangeImage = cv2.inRange(hsv, lowerOrange, upperOrange)
        # whiteImage = cv2.inRange(hsv, lowerWhite, upperWhite)
        
        # orangeImage = close(orangeImage, get_kernal('rect',(7,7)))

        # _, orangeContours, _ = cv2.findContours(orangeImage.copy(), 
        #                                     cv2.RETR_TREE, 
        #                                     cv2.CHAIN_APPROX_SIMPLE)
        # _, whiteContours, _ = cv2.findContours(whiteImage.copy(), 
        #                                     cv2.RETR_TREE, 
        #                                     cv2.CHAIN_APPROX_SIMPLE)
        # xOrange = 0
        # yOrange = 0
        # for c in orangeContours:
        #     rect = (x,y),(ww,hh),angle =cv2.minAreaRect(c)
        #     area = ww*hh
        #     xOrange = x
        #     yOrange = y
        #     box = cv2.boxPoints(rect)
        #     box = np.int0(box)
        #     cv2.drawContours(imageForDraw,[box], -1,(0,0,255),1)

        # xBinCover = 0
        # xBinNonCover = 0
        # yBinCover = 0
        # yBinNonCover = 0

        # for c in whiteContours:
        #     rect = (x,y),(ww,hh),angle =cv2.minAreaRect(c)
        #     area = ww*hh
        #     diff = abs(x - xOrange)
        #     box = cv2.boxPoints(rect)
        #     box = np.int0(box)
        #     cv2.drawContours(imageForDraw,[box], -1,(0,255,0),1)
        #     if diff < 20:
        #         xBinCover = x
        #         yBinCover = y
        #     else:
        #         xBinNonCover = x
        #         yBinNonCover = y
        cv2.imshow('gray', gray)
        # cv2.imshow('hsv', hsv)
        cv2.imshow('image', image)
        cv2.imshow('thresh', thresh)
        cv2.waitKey(30)

=======
        
        imageForDraw = img.copy()
        gamma = adjust_gamma_by_v(image)
        eq = equalization(gamma)
        eq = cv2.cvtColor(eq, cv2.COLOR_HSV2BGR)
        gray = cv2.cvtColor(eq, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,190,255,cv2.THRESH_BINARY)
        dilateImage = dilate(thresh, get_kernal('cross', (31,31)))

        eqOrange = equalization(img.copy())
        # eqOrange = cv2.cvtColor(eqOrange, cv2.COLOR_HSV2BGR)
        eqOrange = cv2.inRange(eqOrange, lowerOrange, upperOrange)
        # eqOrange = cv2.cvtColor(eq, cv2.COLOR_HSV2BGR)
        
        orangeImage = close(eqOrange, get_kernal('rect',(15,15)))

        _, orangeContours, _ = cv2.findContours(orangeImage.copy(), 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)
        _, whiteContours, _ = cv2.findContours(dilateImage.copy(), 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)
        xOrange = 0
        yOrange = 0
        xBinCover = 0
        xBinNonCover = 0
        yBinCover = 0
        yBinNonCover = 0
        for c in orangeContours:
            rect = (x,y),(ww,hh),angle =cv2.minAreaRect(c)
            area = ww*hh
            if area < 500:
                continue
            xOrange = x
            yOrange = y
            xBinCover = x
            yBinCover = y
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(imageForDraw,[box], -1,(0,0,255),3)


        for c in whiteContours:
            rect = (x,y),(ww,hh),angle =cv2.minAreaRect(c)
            area = ww*hh
            if area < 5000:
                continue
            # print('area',area)
            diff = abs(x - xOrange)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(imageForDraw,[box], -1,(0,255,0),3)
            if diff < 20:
                xBinCover = x
                yBinCover = y
            else:
                xBinNonCover = x
                yBinNonCover = y
        print('xCov: ', xBinCover, 'yCov: ', yBinCover)
        print('xNonCov: ', xBinNonCover, 'yNonCov: ', yBinNonCover)

        if msg == NonCov:
            res.x = xBinNonCover
            res.y = yBinNonCover
        else:
            res.x = xBinCover
            res.y = yBinCover
        publish_result(imageForDraw, 'bgr', debug)

        return res
        # cv2.imshow('eqOrange', orangeImage)
        # cv2.imshow('eq', eq)
        # cv2.imshow('dilateImage', dilateImage)
        # cv2.imshow('imageForDraw', imageForDraw)
        # cv2.waitKey(30)

def mission_callback(msg):
    return find_bin(msg.req.data)
>>>>>>> bd55591b68580b349afad1141eda1399efbbdd74


def img_callback(msg):
    global img, width, height
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (640, 512))

    height, width, _ = img.shape
    

if __name__ == '__main__':
    rospy.init_node('findPath')
    bag = '/leftcam_bottom/image_raw/compressed'
    topic = '/bottom/left/image_raw/compressed'
    bot = '/bottom/left/image_raw/compressed'
    rospy.Subscriber(bag, CompressedImage, img_callback)
<<<<<<< HEAD
    find_bin()
=======
    rospy.Service('vision_bin', vision_srv_default(), mission_callback)
    rospy.spin()
    # find_bin()
>>>>>>> bd55591b68580b349afad1141eda1399efbbdd74
