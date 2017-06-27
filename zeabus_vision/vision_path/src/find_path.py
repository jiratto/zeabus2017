#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
import math
from sensor_msgs.msg import CompressedImage
from vision_lib import *
from zeabus_vision_srv_msg.msg import vision_msg_default
from zeabus_vision_srv_msg.srv import vision_srv_default

img = None
# width = 1280
# height = 1024

lower_red = np.array([ 10, 47, 27])
upper_red = np.array([ 23, 255, 255])

def find_path():
    global img, width, height
    print("1")
    t = True
    f = False
    cnt = None
    kernel1 = np.ones((15,15), np.uint8)
    kernel2 = np.ones((5,5), np.uint8)
    res = vision_msg_default()

    # while not rospy.is_shutdown():
    while img is None:
        print("img: None")
        rospy.sleep(0.01)
        # continue
    print("2")
    im = img.copy()
    height, width,_ = im.shape
    # print('width', width)
    # print('height', height)
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
    im_for_draw = np.zeros((height, width))
    im_blur = cv2.GaussianBlur(im, (3,3), 0)
    hsv = cv2.cvtColor(im_blur, cv2.COLOR_BGR2HSV)
    imgray = cv2.cvtColor(im_blur, cv2.COLOR_BGR2GRAY)
    im_red = cv2.inRange(hsv, lower_red, upper_red)
    dilation = cv2.dilate(im_red, kernel1, iterations =  1)
    erosion = cv2.erode(dilation, kernel2, iterations = 1)
    ret, thresh = cv2.threshold(imgray, 200, 255, 0)
    _, contours, hierarchy = cv2.findContours(dilation.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        M = cv2.moments(c)
        rect = (x,y),(ww,hh),angle1 =cv2.minAreaRect(c)
        area = ww*hh
        if area < 500:
            continue
        if hh == 0 :
            continue
        if ww > hh:
            tmp = hh
            hh = ww
            ww = tmp
        diff = (ww/hh)
        # if diff > 0.3:
        #     continue
        epsilon = 0.1*cv2.arcLength(c, t)
        approx = cv2.approxPolyDP(c, epsilon,t)
        
        if area > max1:
            max1 = area
            # cnt = c
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            angle = 90-Oreintation(M)[0]*180/math.pi
            cx = x
            cy = y
            # print('cx',cx)
            # print('cy',cy)
            w = ww
            h = hh
        # cv2.drawContours(im_for_draw,[box], -1,(0,0,255),1)
        cv2.drawContours(im, [approx], 0, (0, 0, 255), 2)
    print('3')
    cv2.circle(im_for_draw,(int(cx), int(cy)), 5, (0, 0, 255), -1)
    cv2.drawContours(im_for_draw,[box], -1,(0,0,255),1)
    cv2.drawContours(im, contours, -1, (0,255,0), 3)
    xx = (cx-offsetW)/offsetW
    yy = (offsetH-cy)/offsetH
    res.x = yy
    res.y = -xx
    res.area = max1
    res.appear = True
    res.angle = angle
    print('max1',max1)
    if max1<500:
        xx = -999
        yy = -999
        res.appear = False
        res.x = -999
        res.y = -999
        res.area = -999
        res.angle = -999
    print('x', yy)
    print('y', -xx)
    print('max',max1)
    print('angle', angle)
    # cv2.imshow('img',img)
    print("4")
    # cv2.waitKey(0)
    # cv2.destroyAllWindow()
    # k = cv2.waitKey(1) & 0xff
    # print('kuykuy')
    # if k == ord('q'):
    #     rospy.signal_shutdown('')
    return res
    
def img_callback(msg):
    global img
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (640, 512))
 


def mission_callback(msg):
    print('mission_callback')
    return find_path()

if __name__ == '__main__':
    rospy.init_node('findPath1')
    topic = '/leftcam_bottom/image_raw/compressed'
    rospy.Subscriber(topic, CompressedImage, img_callback)
    # find_path()
    # while not rospy.is_shutdown():
    #     res = vision_srv_default()
    #     find_path()
    #     print("555")
    rospy.Service('vision', vision_srv_default(), mission_callback)
    rospy.spin()