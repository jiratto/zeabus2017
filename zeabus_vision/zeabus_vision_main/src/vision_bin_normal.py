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
    # img = adjust_gamma_by_v(img)


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
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        gray = cv2.equalizeHist(gray)
        kernel = get_kernal('plus', (3, 3))
        gray = dilate(gray, kernel)
        # gray = cv2.equalizeHist(gray)
        # gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)

        # mean = int(cv2.mean(v)[0])
        # mode = get_mode(gray)
        # print 'mean: ' + str(mode)
        # if mode >= 240:
        #     mode = 200
        # mode += 10
        # print 'mode: ' + str(mode)
        th = cv2.adaptiveThreshold(gray, 220, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU, 13, 2)
        # kernel = get_kernal('plus', (3, 3))
        # th = erode(th, kernel)
        # kernel = get_kernal('rect', (3, 3))
        # th = dilate(th, kernel)
        res = np.zeros((height, width))
        _, contours, hierarchy = cv2.findContours(
            th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            m = cv2.moments(c)
            area = cv2.contourArea(c)

            #     continue
            if not (find_shape(c, 4) or find_shape(c, 5) or find_shape(c, 6)):
                continue
            elif area <= (width * height) / divideArea:
                continue
            elif cut_contours(m, width, height, 20, 20):
                continue
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            (x, y), (w, h), angle = rect
            cv2.drawContours(img, [box], 0, (0, 255, 0), 2)
            cv2.drawContours(img, [c], 0, (0, 0, 255), 2)
        cv2.imshow('gray', gray)
        cv2.imshow('result', res)
        cv2.imshow('threshold', th.copy())
        cv2.imshow('image', img.copy())
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
