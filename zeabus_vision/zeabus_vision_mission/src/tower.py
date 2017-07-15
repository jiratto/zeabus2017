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

img = None
width = None
height = None

def find_tower():
    global img, width, height;

    lower_red, upper_red = get_color('red', 'bottom', 'tower')
    lower_white, upper_white = get_color('white', 'bottom', 'tower')
    lower_green, upper_green = get_color('black', 'bottom', 'tower') # store green color in black param
    lower_orange, upper_orange = get_color('orange', 'bottom', 'tower')
    lower_blue, upper_blue = get_color('yellow', 'bottom', 'tower') # store blue color in yellow param

    while not rospy.is_shutdown():
        while img is None:
            print('none')
        
        image = img.copy()
        imageForDraw = image.copy()

        bgr = preprocess_tower(image)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        orange = cv2.inRange(hsv, lower_orange, upper_orange)
        blue = cv2.inRange(hsv, lower_blue, upper_blue)
        green = cv2.inRange(hsv, lower_green, upper_green)
        red = cv2.inRange(hsv, lower_red, upper_red)

        orange = cv2.subtract(orange, blue)
        red = cv2.subtract(red, orange)

        blue = open_morph(blue, get_kernal('cross', (3,3)))
        
        orange = close(orange, get_kernal('cross', (3,3)))
        orange = open_morph(orange, get_kernal('cross', (3,3)))

        green = close(green, get_kernal('cross', (3,3)))
        green = open_morph(green, get_kernal('cross', (3,3)))

        red = close(red, get_kernal())
        red = open_morph(red, get_kernal('cross', (3,3)))


        _, blueContours, hierarchy = cv2.findContours(blue.copy(), 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)

        _, orangeContours, hierarchy = cv2.findContours(orange.copy(), 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)

        _, greenContours, hierarchy = cv2.findContours(green.copy(), 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)

        _, redContours, hierarchy = cv2.findContours(red.copy(), 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)

        minArea = 300
        maxBlue = 0
        maxOrange = 0
        maxGreen = 0
        maxRed = 0

        boxBlue = None
        boxOrange = None
        boxGreen = None
        boxRed = None

        for c in blueContours:
            rect = (x,y), (ww,hh), angle = cv2.minAreaRect(c)
            area = ww*hh
            if area < minArea:
                continue
            if maxBlue < area:
                maxBlue = area
                boxBlue = cv2.boxPoints(rect)
                boxBlue = np.int0(boxBlue)
        if maxBlue != 0:
            cv2.drawContours(imageForDraw, [boxBlue], -1, (255,255,0), 2)

        for c in orangeContours:
            rect = (x,y), (ww,hh), angle = cv2.minAreaRect(c)
            area = ww*hh
            if area < minArea:
                continue
            if maxOrange < area:
                maxOrange = area
                boxOrange = cv2.boxPoints(rect)
                boxOrange = np.int0(boxOrange)
        if maxOrange != 0:
            cv2.drawContours(imageForDraw, [boxOrange], -1, (0,100,255), 2)

        for c in greenContours:
            rect = (x,y), (ww,hh), angle = cv2.minAreaRect(c)
            area = ww*hh
            if area < minArea:
                continue
            if maxGreen < area:
                maxGreen = area
                boxGreen = cv2.boxPoints(rect)
                boxGreen = np.int0(boxGreen)
        if maxGreen != 0:
            cv2.drawContours(imageForDraw, [boxGreen], -1, (0,255,0), 2)

        for c in redContours:
            rect = (x,y), (ww,hh), angle = cv2.minAreaRect(c)
            area = ww*hh
            if area < minArea:
                continue
            if maxRed < area:
                maxRed = area
                boxRed = cv2.boxPoints(rect)
                boxRed = np.int0(boxRed)
        if maxRed != 0:
            cv2.drawContours(imageForDraw, [boxRed], -1, (0,0,255), 2)

        cv2.imshow('blue', blue)
        cv2.imshow('orange', orange)
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
    find_tower()