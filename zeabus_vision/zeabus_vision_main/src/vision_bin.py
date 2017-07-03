#!/usr/bin/env python
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from vision_lib import *
width = 480
height = 320
img = None


def camera_callback(ros_data):
    global img, width, height
    np_arr = np.fromstring(ros_data.data, np.uint8)
    img = cv2.resize(cv2.imdecode(
        np_arr, 1), (width, height))
    img = adjust_gamma_by_v(img)


def bin_detector():
    global img, width, height
    divideArea = 100
    while img is None:
        print 'image is None 1'
        rospy.sleep(0.1)

    while not rospy.is_shutdown():
        if img is None:
            print 'image is None 2'
            continue

        gray = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2GRAY)
        # gray = cv2.GaussianBlur(gray, (5, 5), 0)
        gray = cv2.equalizeHist(gray)

        # gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)
        hsv = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        # mean = int(cv2.mean(v)[0])
        mode = get_mode(gray)
        print 'mean: ' + str(mode)
        if mode >= 240:
            mode = 200
        mode += 10
        print 'mode: ' + str(mode)
        th = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 11, 2)
        kernel = get_kernal('rect', (1, 1))
        th = erode(th, kernel)
        # kernel = get_kernal('rect', (5, 5))
        # th = dilate(th, kernel)
        # ret3, th = cv2.threshold(
        #     gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # ret, th = cv2.threshold(
        #     gray.copy(), mode, 255, cv2.THRESH_BINARY)
        res = np.zeros((height, width))
        im, contours, hierarchy = cv2.findContours(
            th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            m = cv2.moments(c)
            area = cv2.contourArea(c)
            if area <= (width * height) / divideArea:
                continue
            # if cut_contours(m, width, height, 10, 10) and not (area <= (width * height) / divideArea):
            #     continue
            if not (find_shape(c, 4) or find_shape(c, 5)):
                continue
            cv2.drawContours(res, [c], 0, (255, 255, 255), 2)
        # cv2.imshow('equ', equ)
        cv2.imshow('gray', gray)
        cv2.imshow('result', res)
        cv2.imshow('threshold', th.copy())
        # cv2.imshow('image', img.copy())
        k = cv2.waitKey(1) & 0xff
        if k == ord('q'):
            break
        rospy.sleep(0.1)
    cv2.destroyAllWindows()
if __name__ == '__main__':
    rospy.init_node('vision_bin')
    topic = '/bottom/left/image_raw/compressed'
    sub = rospy.Subscriber(topic, CompressedImage, camera_callback)
    bin_detector()
