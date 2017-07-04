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

def find_bin(msg):
    global img, width, height
    req = msg.req.data
    print('req', req)
    lowerOrange, upperOrange = getColor('orange', 'down')
    lowerWhite, upperWhite = getColor('white', 'down')
    res = vision_msg_default()

    while not rospy.is_shutdown():
        while img is None:
            print('None')

        offsetW = width/2
        offsetH = height/2

        image = img.copy()
        

        imageForDraw = img.copy()
        
        # hsv = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2HSV)
        cla = clahe(image)
        cla = cv2.cvtColor(cla, cv2.COLOR_HSV2BGR)
        hsv = equalization(cla)
        # hsv = equalization(image.copy())
        whiteImage = cv2.inRange(hsv, lowerWhite, upperWhite)
        whiteImage = close(whiteImage, get_kernal())
        # whiteImage = close(whiteImage, get_kernal())
        
        gamma = adjust_gamma_by_v(image)
        eq = equalization(gamma)
        eq = cv2.cvtColor(eq, cv2.COLOR_HSV2BGR)
        gray = cv2.cvtColor(eq, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,235,255,cv2.THRESH_BINARY)
        dilateImage = dilate(thresh, get_kernal('cross', (31,31)))

        eqOrange = equalization(img.copy())
        # eqOrange = cv2.cvtColor(eqOrange, cv2.COLOR_HSV2BGR)
        eqOrange = cv2.inRange(eqOrange, lowerOrange, upperOrange)
        # eqOrange = cv2.cvtColor(eq, cv2.COLOR_HSV2BGR)
        
        orangeImage = close(eqOrange, get_kernal('rect',(15,15)))

        _, orangeContours, _ = cv2.findContours(orangeImage.copy(), 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)
        _, whiteContours, hierarchy = cv2.findContours(whiteImage.copy(), 
                                            cv2.RETR_EXTERNAL, 
                                            cv2.CHAIN_APPROX_SIMPLE)
        xOrange = -999
        yOrange = -999
        xBinCover = -999
        xBinNonCover = -999
        yBinCover = -999
        yBinNonCover = -999
        angle = -999
        countBin = 0
        boxNoCover = image.copy().fill(0)
        boxCover = image.copy().fill(0)
        for c in orangeContours:
            rect = (x,y),(ww,hh),_ =cv2.minAreaRect(c)
            area = ww*hh
            if area < 500:
                continue
            countBin += 1
            xOrange = x
            yOrange = y
            xBinCover = x
            yBinCover = y
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # cv2.drawContours(imageForDraw,[box], -1,(0,0,255),3)
        binAppear = False
        noCover = False
        maxNoCover = 0
        for c in whiteContours:
            M = cv2.moments(c)
            rect = (x,y),(ww,hh),_ =cv2.minAreaRect(c)
            area = ww*hh
            if area < 7000:
                continue
            # print('area',area)
            countBin += 1
            diff = abs(x - xOrange)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            if diff < 20:
                xBinCover = x
                yBinCover = y
                angle = 90-Oreintation(M)[0]*180/math.pi
                binAppear = True
                boxCover = box
            else:
                if maxNoCover < area:
                    maxNoCover = area
                    xBinNonCover = x
                    yBinNonCover = y
                    boxNoCover = box
                    angle = 90-Oreintation(M)[0]*180/math.pi
                    noCover = True
        cv2.drawContours(imageForDraw,[boxNoCover], -1,(0,255,0),3)
        cv2.drawContours(imageForDraw, [boxCover], -1, (255,255,0),3)
        print('xCov: ', xBinCover, 'yCov: ', yBinCover)
        print('xNonCov: ', xBinNonCover, 'yNonCov: ', yBinNonCover)
        print('angle', angle)
        if req == 'nocover': # **Note** swap x and y for AI 
            cv2.circle(imageForDraw ,(int(xBinNonCover), int(yBinNonCover)), 5, (0, 255, 255), -1)
            res.y = -(xBinNonCover-offsetW)/offsetW
            res.x = (offsetH-yBinNonCover)/offsetH
            res.angle = angle
            res.appear = noCover
        elif req == 'bin':
            cv2.circle(imageForDraw ,(int(xBinCover), int(yBinCover)), 5, (0, 255, 255), -1)
            res.y = -(xBinCover-offsetW)/offsetW
            res.x = (offsetH-yBinCover)/offsetH
            res.angle = angle
            res.appear = binAppear
        else:
            print('error no req')
        print('res.x: ', res.x)
        print('res.y', res.y)
        # if countBin == 0:
        #     res.appear = False
        # else:
        #     res.appear = True
        publish_result(imageForDraw, 'bgr', 'debug')
        publish_result(gray, 'gray', 'debug_gray')
        publish_result(thresh, 'gray', 'debug_thresh')
        publish_result(eq, 'bgr', 'debug_eq')
        publish_result(whiteImage, 'gray', 'white')


        return res
        # cv2.imshow('eqOrange', orangeImage)
        # cv2.imshow('eq', eq)
        # cv2.imshow('dilateImage', dilateImage)
        # cv2.imshow('imageForDraw', imageForDraw)
        # cv2.waitKey(30)

def mission_callback(msg):
    print(msg.req.data)
    return find_bin(msg)

def img_callback(msg):
    global img, width, height
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (640, 512))

    height, width, _ = img.shape
    

if __name__ == '__main__':
    rospy.init_node('findBin')
    bag = '/leftcam_bottom/image_raw/compressed'
    topic = '/bottom/left/image_raw/compressed'
    bot = '/bottom/left/image_raw/compressed'
    rospy.Subscriber(bot, CompressedImage, img_callback)
    # rospy.Subscriber(bag, CompressedImage, img_callback)
    # find_bin()
    rospy.Service('vision_bin', vision_srv_default(), mission_callback)
    rospy.spin()
    # find_bin()
