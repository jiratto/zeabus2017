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

def find_bin():
    global img

    lowerOrange, upperOrange = getColor('orange', 'down')
    lowerWhite, upperWhite = getColor('white', 'down')

    while not rospy.is_shutdown():
        while img is None:
            print('None')
        image = img.copy()
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
    find_bin()