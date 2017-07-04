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

def adjust_gamma2(image, gamma=1.0):
    invGamma = 1.0/gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
    for i in np.arange(0, 256)]).astype("uint8")

    return cv2.LUT(image, table)

def canny(image, threshold1, threshold2):
    return cv2.Canny(image, threshold1, threshold2)

def find_bin():
    global img, width, height
    lowerOrange, upperOrange = getColor('orange', 'down')
    lowerWhite, upperWhite = getColor('white', 'down')

    while not rospy.is_shutdown():
        while img is None:
            print('None')

        print('lowerWhite', lowerWhite)
        print('upperWhite', upperWhite)
        offsetW = width/2
        offsetH = height/2

        image = img.copy()
        imageForDraw = img.copy()

        edge = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2GRAY)
        edge = cv2.Canny(edge, 100, 150)

        gamma2 = img.copy()
        gamma2 = adjust_gamma(gamma2, 4)
        # gamma2 = cv2.GaussianBlur(gamma2, (15,15), 0)
        # gamma2 = equalization(gamma2)
        # gamma2 = cv2.cvtColor(gamma2, cv2.COLOR_HSV2BGR)
        blur = cv2.bilateralFilter(gamma2,9,75,75)

        grayScaleImage = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2GRAY)/255
        print('grayScaleImage', grayScaleImage)
        # hsv = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2HSV)
        hsv = equalization(image.copy())
        whiteImage = cv2.inRange(gamma2, lowerWhite, upperWhite)
        whiteImage = close(whiteImage, get_kernal())
        # whiteImage = close(whiteImage, get_kernal())
        
        gamma = adjust_gamma_by_v(image)
        eq = equalization(img.copy())
        eq = cv2.cvtColor(eq, cv2.COLOR_HSV2BGR)
        median = cv2.medianBlur(eq.copy(),49)
        median = cv2.cvtColor(median, cv2.COLOR_BGR2GRAY)
        # median = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)
        # thresh3 = eq
        thresh3 = cv2.cvtColor(eq.copy(), cv2.COLOR_BGR2GRAY)
        _,thresh3 = cv2.threshold(median,190,255,cv2.THRESH_BINARY_INV)
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
        _, whiteContours, _ = cv2.findContours(whiteImage.copy(), 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)
        xOrange = -999
        yOrange = -999
        xBinCover = -999
        xBinNonCover = -999
        yBinCover = -999
        yBinNonCover = -999
        angle = -999
        countBin = 0
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
        testArea = 0
        boxNoCover = image.copy().fill(0)
        # _,grayScaleImage = cv2.threshold(grayScaleImage,127,255,cv2.THRESH_BINARY)
        for c in whiteContours:
            M = cv2.moments(c)
            rect = (x,y),(ww,hh), debugArea =cv2.minAreaRect(c)
            area = ww*hh
            if area < 7000:
                continue
            cv2.drawContours(imageForDraw, [c], -1,(255,255,0), 1)
            cv2.drawContours(grayScaleImage, [c], -1, (255, 255, 255), 3)
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
            else:
                if maxNoCover < area:
                    maxNoCover = area
                    testArea = 90-debugArea
                    xBinNonCover = x
                    yBinNonCover = y
                    boxNoCover = box
                    angle = 90-Oreintation(M)[0]*180/math.pi
                    noCover = True
        cv2.drawContours(imageForDraw,[boxNoCover], -1,(0,255,0),3)
        print('xCov: ', xBinCover, 'yCov: ', yBinCover)
        print('xNonCov: ', xBinNonCover, 'yNonCov: ', yBinNonCover)
        print('testAngle',testArea)
        print('angle', angle)

        grayScaleImage = close(grayScaleImage, get_kernal('rect', (21,21)))

        cv2.imshow('median', median)
        cv2.imshow('thresh3', thresh3)
        # cv2.imshow('grayScaleImage', grayScaleImage)
        # cv2.imshow('canny edge', edge)
        # cv2.imshow('0gamma2', gamma2)
        # cv2.imshow('median', median)
        # cv2.imshow('blur', blur)
        # cv2.imshow('whiteImage', whiteImage)
        # cv2.imshow('imageForDraw', imageForDraw)
        # cv2.imshow('thresh', thresh)
        # cv2.imshow('gamma', gamma)
        # cv2.imshow('eq', eq)
        # cv2.imshow('gray', gray)
        cv2.waitKey(30)
        
        

def mission_callback(msg):
    print(msg.req.data)
    return find_bin(msg)


def img_callback(msg):
    global img, width, height
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (640, 512))

    height, width, _ = img.shape
    

if __name__ == '__main__':
    rospy.init_node('debug')
    bag = '/leftcam_bottom/image_raw/compressed'
    topic = '/bottom/left/image_raw/compressed'
    bot = '/bottom/left/image_raw/compressed'
    rospy.Subscriber(bot, CompressedImage, img_callback)
    # rospy.Service('vision_bin', vision_srv_default(), mission_callback)
    # rospy.spin()
    find_bin()