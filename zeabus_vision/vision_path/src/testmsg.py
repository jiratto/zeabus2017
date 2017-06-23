#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
import math
import sys
sys.path.append ('/home/zeabus/catkin_ws/src/src_code/zeabus_vision/main/src/')
from vision_lib import *
from sensor_msgs.msg import CompressedImage

img = None

lower_red = np.array([ 10, 47, 27])
upper_red = np.array([ 23, 255, 255])


def find_path():
    global img, width, height
    lo, uo = getColor('yellow', 'top')
    t = True
    f = False
    cnt = None
    kernel1 = np.ones((15,15), np.uint8)
    kernel2 = np.ones((5,5), np.uint8)

    while not rospy.is_shutdown():
        while img is None:
            print("img: None")
            rospy.sleep(0.01)
            # continue
        im = img.copy()
        height, width,_ = im.shape
        offsetW = width/2
        offsetH = height/2
        area = -999
        max1 = -999
        cx = -999
        cy = -999
        w = -999
        h = -999
        angle = -999
        box = None
        imStretching = stretching(im)
        im_for_draw = img.copy()
        im_blur = cv2.GaussianBlur(im, (3,3), 0)
        hsv = cv2.cvtColor(im_blur, cv2.COLOR_BGR2HSV)
        imgray = cv2.cvtColor(im_blur, cv2.COLOR_BGR2GRAY)
        im_red = cv2.inRange(hsv, lo, uo)
        dilation = cv2.dilate(im_red, kernel1, iterations =  1)
        
        erosion = cv2.erode(dilation, kernel2, iterations = 1)
        ret, thresh = cv2.threshold(imgray, 200, 255, 0)
        _, contours, hierarchy = cv2.findContours(dilation.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            M = cv2.moments(c)
            rect = (x,y),(ww,hh),angle1 =cv2.minAreaRect(c)
            area = ww*hh
            if area < 2000:
                continue
            if hh == 0 :
                continue
            if ww > hh:
                tmp = hh
                hh = ww
                ww = tmp
            diff = (ww/hh)
            epsilon = 0.1*cv2.arcLength(c, t)
            approx = cv2.approxPolyDP(c, epsilon,t)
            # box = cv2.boxPoints(rect)
            # box = np.int0(box)
            # cv2.drawContours(im_for_draw,[box], -1,(0,0,255),1)
            
            if area > max1:
                max1 = area
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                # cv2.drawContours(im_for_draw,[box], -1,(0,0,255),1)
                angle = 90-Oreintation(M)[0]*180/math.pi
                cx = x
                cy = y
                w = ww
                h = hh
            cv2.drawContours(im, [approx], 0, (0, 0, 255), 2)
        cv2.circle(im_for_draw,(int(cx), int(cy)), 5, (0, 0, 255), -1)
        cv2.drawContours(im_for_draw,[box], -1,(0,0,255),1)
        cv2.drawContours(im, contours, -1, (0,255,0), 3)
        xx = (cx-offsetW)/offsetW
        yy = (offsetH-cy)/offsetH
        cv2.imshow('img',im_for_draw)
        cv2.imshow('dilate',dilation)
        cv2.imshow('imred', im_red)
        cv2.waitKey(30)
        
        
def img_callback(msg):
    global img
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (640, 512))
    
    
    
def mission_callback(msg):
    print('mission_callback')
    return find_path()

if __name__ == '__main__':
    rospy.init_node('findPath')
    topic = '/rightcam_bottom/image_raw/compressed'
    top = '/leftcam_top/image_raw/compressed'
    rospy.Subscriber(top, CompressedImage, img_callback)
    find_path()