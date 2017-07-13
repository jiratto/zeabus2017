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

        cla = clahe(image)
        blur = cv2.bilateralFilter(cla, 15, 75, 75)
        blurClaGray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        blurClaGray = equalization_gray(blurClaGray)
        
        ret, black = cv2.threshold(blurClaGray, 30, 255, cv2.THRESH_BINARY_INV)
        # ret, black1 = cv2.threshold(blurClaGray, 200, 255, cv2.THRESH_BINARY)
        # black = np.invert(black1)
        # black = erode(black, get_kernal('cross', (13,13)))

        eqOrange = clahe(image)
        eqOrange = cv2.cvtColor(eqOrange, cv2.COLOR_BGR2HSV)
        eqOrange = equalization_hsv(eqOrange)
        eqOrange = cv2.inRange(eqOrange, lowerOrange, upperOrange)
        
        orangeImage = close(eqOrange, get_kernal('rect',(15,15)))

        _, orangeContours, _ = cv2.findContours(orangeImage.copy(), 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)
        _, blackContours, hierarchy = cv2.findContours(black.copy(), 
                                            cv2.RETR_EXTERNAL, 
                                            cv2.CHAIN_APPROX_SIMPLE)
        xBinCover = -999
        xBinNonCover = -999
        
        yBinCover = -999
        yBinNonCover = -999
        
        boxNoCover = image.copy().fill(0)
        boxCover = image.copy().fill(0)
        
        maxNoCover = 0
        maxCover = 0
        
        range_w = 25
        range_h = 25
        
        binAppear = False
        noCoverAppear = False
        
        noCoverAngle = -999
        coverAngle = -999

        offsetShootX = 25
        offsetShootY = 25
        for c in orangeContours:
            M = cv2.moments(c)
            rect = (x,y),(ww,hh),_ =cv2.minAreaRect(c)
            area = ww*hh
            if not find_shape(c, 'rect'):
                continue
            if area < 500:
                continue
            if maxCover < area:
                maxCover = area
                xBinCover = x
                yBinCover = y
                boxCover = cv2.boxPoints(rect)
                boxCover = np.int0(boxCover)
                coverAngle = 90-Oreintation(M)[0]*180/math.pi
        
        cv2.drawContours(imageForDraw,[boxCover], -1,(0,0,255),3)
        
        for c in blackContours:
            M = cv2.moments(c)
            rect = (x,y),(ww,hh),_ =cv2.minAreaRect(c)
            area = ww*hh
            diff = abs(x-xBinCover)
            if diff < 20:
                continue
            if not find_shape(c, 'rect'):
                continue
            if area < 1000:
                continue
            if cut_contours(M, width, height, range_w, range_h):
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(imageForDraw, [c], -1, (255,255,0), 2)
                continue
            print('area', area)
            # print('area',area)
            
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(imageForDraw, [c], -1, (255,0,0), 2)
            
            if maxNoCover < area:
                maxNoCover = area
                xBinNonCover = x
                yBinNonCover = y
                boxNoCover = box
                noCoverAngle = 90-Oreintation(M)[0]*180/math.pi
                noCoverAppear = True

        if maxCover > 0:
            binAppear = True
        if maxNoCover > 0:
            noCoverAppear = True

        cv2.drawContours(imageForDraw,[boxNoCover], -1,(0,255,0),3)
        cv2.drawContours(imageForDraw, [boxCover], -1, (255,255,0),3)

        print('angle', noCoverAngle)

        xBinNonCover -= offsetShootX
        # yBinNonCover += offsetShootY

        if req == 'nocover': # **Note** swap x and y for AI 
            cv2.circle(imageForDraw ,(int(xBinNonCover), int(yBinNonCover)), 5, (0, 255, 255), -1)
            res.y = -(xBinNonCover-offsetW)/offsetW
            res.x = (offsetH-yBinNonCover)/offsetH
            res.angle = noCoverAngle
            res.appear = noCoverAppear
        elif req == 'bin':
            cv2.circle(imageForDraw ,(int(xBinCover), int(yBinCover)), 5, (0, 255, 255), -1)
            res.y = -(xBinCover-offsetW)/offsetW
            res.x = (offsetH-yBinCover)/offsetH
            res.angle = coverAngle
            res.appear = binAppear
        else:
            print('error no req')
        publish_result(imageForDraw, 'bgr', 'debug')
        publish_result(blurClaGray, 'gray', 'blurClaGray')
        publish_result(blur, 'bgr', 'blur')
        publish_result(black, 'gray', 'black')


        return res

def mission_callback(msg):
    print(msg.req.data)
    return find_bin(msg)

def img_callback(msg):
    global img, width, height
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (640/2, 384/2))

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
