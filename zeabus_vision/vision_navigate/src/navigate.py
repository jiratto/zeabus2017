#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
import math
<<<<<<< HEAD
import sys
sys.path.append ('/home/zeabus/catkin_ws/src/src_code/zeabus_vision/main/src/')
from vision_lib import *
=======
<<<<<<< HEAD
import sys
sys.path.append ('/home/zeabus/catkin_ws/src/src_code/zeabus_vision/main/src/')
from vision_lib import *
from sensor_msgs.msg import CompressedImage
from zeabus_vision_srv_msg.msg import vision_msg_navigate
from zeabus_vision_srv_msg.srv import vision_srv_navigate

img = None

lower_yellow, upper_yellow = getColor('yellow', 'top') 

def rect_ker(x,y):
    return cv2.getStructuringElement(cv2.MORPH_RECT,(x,y))

def ellipse_ker(x,y):
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(x,y))

def cross_ker(x,y):
    return cv2.getStructuringElement(cv2.MORPH_CROSS,(x,y))

def erode(pic, ker):
    return cv2.erode(pic, ker, iterations = 1)

def dilate(pic, ker):
    return cv2.dilate(pic, ker, iterations =  1)

def close(pic, ker):
    return cv2.morphologyEx(pic, cv2.MORPH_CLOSE, ker)

def tophat(pic, ker):
    return cv2.morphologyEx(pic, cv2.MORPH_TOPHAT, ker)
=======
>>>>>>> e4c989a32864ba9bcad85eecfbe77728ac9d5bcd
from sensor_msgs.msg import CompressedImage
from zeabus_vision_srv_msg.msg import vision_msg_navigate
from zeabus_vision_srv_msg.srv import vision_srv_navigate

img = None

lower_yellow, upper_yellow = getColor('yellow', 'top') 

def rect_ker(x,y):
    return cv2.getStructuringElement(cv2.MORPH_RECT,(x,y))

def ellipse_ker(x,y):
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(x,y))

<<<<<<< HEAD
def cross_ker(x,y):
    return cv2.getStructuringElement(cv2.MORPH_CROSS,(x,y))

def erode(pic, ker):
    return cv2.erode(pic, ker, iterations = 1)

def dilate(pic, ker):
    return cv2.dilate(pic, ker, iterations =  1)

def close(pic, ker):
    return cv2.morphologyEx(pic, cv2.MORPH_CLOSE, ker)

def tophat(pic, ker):
    return cv2.morphologyEx(pic, cv2.MORPH_TOPHAT, ker)
=======
>>>>>>> 747e5ee4962dc2678b25aedcd864ce4422d17d44
>>>>>>> e4c989a32864ba9bcad85eecfbe77728ac9d5bcd

def navigate():
    global img
    im = None
<<<<<<< HEAD
    res = vision_msg_navigate()
=======
<<<<<<< HEAD
    res = vision_msg_navigate()
    while not rospy.is_shutdown():
        while img is None:
            print("img : None")
        height, width,_ = img.shape
        im_size = height*width
        offsetW = width/2
        offsetH = height/2
        im = img.copy()
        im_draw = img.copy()
        hsv = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
        print('lower_yellow', lower_yellow)
        print('upper_yellow', upper_yellow)
        im_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        im_delnoise = erode(im_yellow, cross_ker(5,5))
        im_lr = dilate(im_delnoise, rect_ker(11,1))
        im_lr = close(im_lr, rect_ker(11,11))
        im_lr = erode(im_lr, rect_ker(1,17))
        im_bot = dilate(im_delnoise, rect_ker(1,15))
        im_bot = erode(im_bot, rect_ker(17, 1))
        plate = im_lr+im_bot
        _, contours_lr, hierarchy = cv2.findContours(im_lr.copy(), 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)

        _, contours_bot, _ = cv2.findContours(im_bot.copy(), 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)

        max = 0
        count_h = 0
        cx = -999
        cy = -999
        lr_x = -999
        area_h = 0
        area_w = 0
        area = -999
        direction = -999
        ratio_area = -999
        x_lr = -999
        x_bot = -999
        for c in contours_lr:
            M = cv2.moments(c)
            rect = (x,y), (ww,hh), angle = cv2.minAreaRect(c)
            area = ww*hh
            if y > 450:
                continue
            if area < 1500:
                continue
            if area_h < ww:
                area_h = ww
            if area_h < hh:
                area_h = hh
            x_lr = x
            count_h += 1
            cy += y
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            print('draw')
            cv2.drawContours(im_draw, [box], -1, (0, 0, 255,), 2) 
            cv2.drawContours(im_draw, c, -1, (0 , 255, 0), 2)
            cv2.circle(im_draw ,(int(x), int(y)), 5, (0, 0, 255), -1)
            verticalX = (x - offsetW)/offsetW
            verticalY = (offsetH - y)/offsetH
        if count_h != 0:
            cy /= count_h

        max = 0
        for c in contours_bot:
            M = cv2.moments(c)
            rect = (x,y), (ww,hh), angle = cv2.minAreaRect(c)
            area = ww*hh
            if area_w < ww:
                area_w = ww
            if area_w < hh:
                area_w = hh
            if area < 3500:
=======
    kernel_1 = np.ones((9,9), np.uint8)
    kernel_2 = np.ones((17,17), np.uint8)
>>>>>>> e4c989a32864ba9bcad85eecfbe77728ac9d5bcd
    while not rospy.is_shutdown():
        while img is None:
            print("img : None")
        height, width,_ = img.shape
        im_size = height*width
        offsetW = width/2
        offsetH = height/2
        im = img.copy()
        im_draw = img.copy()
        hsv = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
        print('lower_yellow', lower_yellow)
        print('upper_yellow', upper_yellow)
        im_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        im_delnoise = erode(im_yellow, cross_ker(5,5))
        im_lr = dilate(im_delnoise, rect_ker(11,1))
        im_lr = close(im_lr, rect_ker(11,11))
        im_lr = erode(im_lr, rect_ker(1,17))
        im_bot = dilate(im_delnoise, rect_ker(1,15))
        im_bot = erode(im_bot, rect_ker(17, 1))
        plate = im_lr+im_bot
        _, contours_lr, hierarchy = cv2.findContours(im_lr.copy(), 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)

        _, contours_bot, _ = cv2.findContours(im_bot.copy(), 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)

        max = 0
        count_h = 0
        cx = -999
        cy = -999
        lr_x = -999
        area_h = 0
        area_w = 0
        area = -999
        direction = -999
        ratio_area = -999
        x_lr = -999
        x_bot = -999
        for c in contours_lr:
            M = cv2.moments(c)
            rect = (x,y), (ww,hh), angle = cv2.minAreaRect(c)
            area = ww*hh
            if y > 450:
                continue
            if area < 1500:
                continue
            if area_h < ww:
                area_h = ww
            if area_h < hh:
                area_h = hh
            x_lr = x
            count_h += 1
            cy += y
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            print('draw')
            cv2.drawContours(im_draw, [box], -1, (0, 0, 255,), 2) 
            cv2.drawContours(im_draw, c, -1, (0 , 255, 0), 2)
            cv2.circle(im_draw ,(int(x), int(y)), 5, (0, 0, 255), -1)
            verticalX = (x - offsetW)/offsetW
            verticalY = (offsetH - y)/offsetH
        if count_h != 0:
            cy /= count_h

        max = 0
        for c in contours_bot:
            M = cv2.moments(c)
            rect = (x,y), (ww,hh), angle = cv2.minAreaRect(c)
            area = ww*hh
<<<<<<< HEAD
            if area_w < ww:
                area_w = ww
            if area_w < hh:
                area_w = hh
            if area < 3500:
=======
            if area < 17000 or area > 150000:
>>>>>>> 747e5ee4962dc2678b25aedcd864ce4422d17d44
>>>>>>> e4c989a32864ba9bcad85eecfbe77728ac9d5bcd
                continue
            print(area)
            if max<area :
                max = area
<<<<<<< HEAD
            x_bot = x
            cx = x_bot
=======
<<<<<<< HEAD
            x_bot = x
            cx = x_bot
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # print('draw')
            cv2.drawContours(im_draw, [box], -1, (0, 0, 255,), 2) 
            cv2.drawContours(im_draw, c, -1, (0 , 255, 0), 2)
            cv2.circle(im_draw ,(int(x), int(y)), 5, (0, 0, 255), -1)

        if count_h == 1:
            if x_bot > x_lr:
                direction = 1
            else:
                direction = -1
        
        
        if count_h != 0 and area_w != 0:
            area = area_h*area_w
        ratio_area = area/im_size
        cv2.circle(im_draw ,(int(cx), int(cy)), 5, (0, 0, 255), -1)
        cx = (cx-offsetW)/offsetW
        cy = (offsetH-cy)/offsetH
        print('cx',cx)
        print('cy',cy)
        print('direction',direction)
        print('ratio_area',ratio_area)
        print('count_h',count_h)
        res.cx = cx
        res.cy = cy
        res.direction = direction
        res.ratioArea = ratio_area
        res.numVertical = count_h
        return res
        # print('count_h', count_h)
        # cv2.imshow('yellow',im_yellow)
        # cv2.imshow('del noise', im_delnoise)
        # cv2.imshow('im_lr', im_lr)
        # cv2.imshow('im_draw', im_draw)
        # cv2.imshow('plate', plate)
        # cv2.waitKey(30)
=======
            # epsilon = 0.1*cv2.arcLength(c, t)
            # approx = cv2.approxPolyDP(c, epsilon,t)
            
>>>>>>> e4c989a32864ba9bcad85eecfbe77728ac9d5bcd
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # print('draw')
            cv2.drawContours(im_draw, [box], -1, (0, 0, 255,), 2) 
            cv2.drawContours(im_draw, c, -1, (0 , 255, 0), 2)
            cv2.circle(im_draw ,(int(x), int(y)), 5, (0, 0, 255), -1)

        if count_h == 1:
            if x_bot > x_lr:
                direction = 1
            else:
                direction = -1
        
<<<<<<< HEAD
        
        if count_h != 0 and area_w != 0:
            area = area_h*area_w
        ratio_area = area/im_size
        cv2.circle(im_draw ,(int(cx), int(cy)), 5, (0, 0, 255), -1)
        cx = (cx-offsetW)/offsetW
        cy = (offsetH-cy)/offsetH
        print('cx',cx)
        print('cy',cy)
        print('direction',direction)
        print('ratio_area',ratio_area)
        print('count_h',count_h)
        res.cx = cx
        res.cy = cy
        res.direction = direction
        res.ratioArea = ratio_area
        res.numVertical = count_h
        return res
        # print('count_h', count_h)
        # cv2.imshow('yellow',im_yellow)
        # cv2.imshow('del noise', im_delnoise)
        # cv2.imshow('im_lr', im_lr)
        # cv2.imshow('im_draw', im_draw)
        # cv2.imshow('plate', plate)
        # cv2.waitKey(30)
=======
        print('max: ', max)
        cv2.imshow('yellow?',im_yellow)
        cv2.imshow('img', img)
        cv2.imshow('closing', closing)
        # cv2.imshow('dilate', dilation)
        cv2.waitKey(30)
>>>>>>> 747e5ee4962dc2678b25aedcd864ce4422d17d44
>>>>>>> e4c989a32864ba9bcad85eecfbe77728ac9d5bcd


def img_callback(msg):
    global img

    arr = np.fromstring( msg.data, np.uint8)
<<<<<<< HEAD
=======
<<<<<<< HEAD
>>>>>>> e4c989a32864ba9bcad85eecfbe77728ac9d5bcd
    img = cv2.resize(cv2.imdecode(arr, 1), (640, 512))

def mission_callback(msg):
    return navigate()
<<<<<<< HEAD
=======
=======
    img = cv2.imdecode(arr, 1)
>>>>>>> 747e5ee4962dc2678b25aedcd864ce4422d17d44
>>>>>>> e4c989a32864ba9bcad85eecfbe77728ac9d5bcd

if __name__ == '__main__':
    rospy.init_node('Navigate')
    bot = '/rightcam_bottom/image_raw/compressed'
<<<<<<< HEAD
    top = '/top/center/image_rect_color/compressed'
    rospy.Subscriber(top, CompressedImage, img_callback)
    rospy.Service('vision_navigate', vision_srv_navigate(), mission_callback)
    rospy.spin()
    # navigate()
=======
<<<<<<< HEAD
    top = '/top/center/image_rect_color/compressed'
    rospy.Subscriber(top, CompressedImage, img_callback)
    rospy.Service('vision_navigate', vision_srv_navigate(), mission_callback)
    rospy.spin()
    # navigate()
=======
    top = '/rightcam_top/image_raw/compressed'
    rospy.Subscriber(top, CompressedImage, img_callback)
    navigate()
>>>>>>> 747e5ee4962dc2678b25aedcd864ce4422d17d44
>>>>>>> e4c989a32864ba9bcad85eecfbe77728ac9d5bcd
