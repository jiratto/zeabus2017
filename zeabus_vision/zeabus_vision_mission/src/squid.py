#!/usr/bin/env python
'''
Copyright @ EmOne (Thailand) Co.Ltd. 2017
Author: Anol Paisal <info@emone.co.th>
Date: 2017/05/15
'''

import sys
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image
sys.path.append(
    '/home/zeabus/catkin_ws/src/src_code/zeabus_vision/zeabus_vision_main/src/')
from vision_lib import *
import math
from std_msgs.msg import String
from zeabus_vision_srv_msg.srv import *
from zeabus_vision_srv_msg.msg import *

img = None
img_gray = None
hsv = None
client = None
wait = False
thresh = 170
lower_h = 33
lower_s = 110
lower_v = 0
upper_h = 255
upper_s = 255
upper_v = 255
lower_r = 0
lower_g = 0
lower_b = 0
upper_r = 255
upper_g = 255
upper_b = 255
gamma = 7


def on_gamma_callback(param):
    global gamma
    gamma = param


def adjust_gamma(image, gamma=1):
    if gamma == 0:
        g = 1.0
    else:
        g = gamma / 10.0
    invGamma = 1.0 / g
    table = np.array([((i / 255.0) ** invGamma) * 255
                      for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)


def process():
    global hsv, img_gray, thresh, lower_h, lower_s, lower_v, upper_h, upper_s, upper_v, lower_r, lower_g, lower_b, upper_r, upper_g, upper_b
    blur = cv2.GaussianBlur(img_gray, (7, 7), 1)
    # cv2.imshow('blur', blur)
	thresh_out = cv2.adaptiveThreshold(
        blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
	# cv2.imshow('thresh out', thresh_out)
	# publish_result(thresh_out, 'gray', '/gray')
# noise removal
    kernel = np.ones((3, 3), np.uint8)
# opening = cv2.morphologyEx(blur, cv2.MORPH_GRADIENT, kernel, iterations
# = 2)
    opening = cv2.morphologyEx(blur, cv2.MORPH_OPEN, kernel, iterations=3)
    # cv2.imshow('opening', opening)

# sure background area
    sure_bg = cv2.dilate(opening, kernel, iterations=3)

    dist = cv2.distanceTransform(opening, cv2.DIST_L2, 3)
    cv2.normalize(dist, dist, 0, 255, cv2.NORM_MINMAX)
    dist = np.uint8(dist)

    # cv2.imshow('sure_bg', sure_bg)
    # cv2.imshow('dist', dist)
    ret, sure_fg = cv2.threshold(
        dist, dist.max() * 0.7, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
 #   sure_fg = cv2.adaptiveThreshold(dist, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    # cv2.imshow('sure_fg', sure_fg)

    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg, sure_fg)
    # cv2.imshow('unknown', unknown)

    # unknown = cv2.subtract(unknown, thresh_out)
    # cv2.imshow('unknown', unknown)
#        laplacian = cv2.Laplacian(sure_bg,cv2.CV_64F)
#        cv2.imshow('laplacian', laplacian)
    # circles = cv2.HoughCircles(unknown ,cv2.HOUGH_GRADIENT,1,20,
    #                         param1=50,param2=30,minRadius=0,maxRadius=0)

    im2, contours, hierarchy = cv2.findContours(
        unknown, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#    hull = []
    minRect = []

    drawing = hsv.copy()
    hsv_drawing = hsv.copy()
#   for i in range(len(contours)):
#	hull.append(cv2.convexHull(contours[i]))
    ret = []
    for i in range(len(contours)):
        minRect.append(cv2.minAreaRect(contours[i]))
#        if len(contours[i]) < 5:
#       	    minRect.append(cv2.minAreaRect(contours[i]))
#            minEllipse.append(cv2.fitEllipse(contours[i]))
#            cv2.ellipse(drawing, minEllipse[i], (255,0,0), 2)
#            cv2.drawContours(drawing, contours, int(i), (0,255,0), 0)

    cv2.drawContours(drawing, contours, -1, (0, 0, 255), cv2.FILLED)
    # cv2.imshow('Contour', drawing)
    for i in range(len(contours)):
        #	if contours[i].size > 100:
        #        cv2.drawContours(drawing, contours, i, (0,255,0), 1)
        #        cv2.drawContours(drawing, hull, i, (0,255,0), 0)

        box = cv2.boxPoints(minRect[i])
        box = np.int0(box)
        if box[1][1] < box[3][1] and box[0][0] < box[2][0]:
            img_roi = hsv[int(box[1][1]):int(box[3][1]),
                          int(box[0][0]):int(box[2][0])]
            img_roi = cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY)
            # cv2.imshow('ROI', img_roi)
            # create a CLAHE object (Arguments are optional).
            clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(3, 3))
            cl1 = clahe.apply(img_roi)
            res = np.hstack((img_roi, cl1))

            # equ = cv2.equalizeHist(img_roi)
            # res = np.hstack((img_roi, img_roi))
            # cv2.imshow('equ', res)
#            lower_blue = np.array([110,50,50])
#            upper_blue = np.array([130,255,255])
#            mask = cv2.inRange(img_roi, lower_blue, upper_blue)
#            res = cv2.bitwise_and(img_roi,img_roi, mask= mask)
            circles = cv2.HoughCircles(
                img_roi, cv2.HOUGH_GRADIENT, 1.5, 100, param1=80, param2=20, minRadius=0, maxRadius=0)
            if circles != None:
                for i in circles[0, :]:
                    # draw the outer circle
                    # cv2.imshow('ROI', img_roi)
                    cv2.circle(hsv_drawing, (int(
                        box[1][0] + i[0]), int(box[1][1] + i[1])), i[2], (0, 255, 0), 2)
            # draw the center of the circle
#                    cv2.circle(hsv,(i[0],i[1]),2,(0,0,255),3)
                    cv2.circle(
                        hsv_drawing, (int(box[1][0] + i[0]), int(box[1][1] + i[1])), 2, (0, 0, 255), 3)
                    publish_result(hsv_drawing, 'bgr', '/circle')
					# cv2.imshow('circle', hsv_drawing)
                    # return (int(box[1][0]+i[0]), int(box[1][1]+i[1]), int(i[2]))
#                    cv2.imshow('circle', hsv)
#                    cv2.imshow('ROI', img_roi)
                    ret.append({"width": img_gray.shape[1], "height": img_gray.shape[0], "origin_x": int(box[1][0] + i[0]),
                                "origin_y": int(box[1][1] + i[1]), "radius": int(i[2])})
                    # buoy = "window_x={7},window_y={8},origin_x={0},origin_y={1},radius={2},prob_blue={3},prob_green={4},prob_red={5},color={6}" \
                    #             .format(ret[i][0], ret[i][1], ret[i][2], ret[i][3], ret[i][4], ret[i][5], ret[i][6],  img_gray.shape[1], img_gray.shape[0])
    print("Count all: {}".format(len(ret)))
    rospy.loginfo(ret)
    return ret


def callback(msg):
	global img, img_gray, hsv, gamma
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecode(arr, 1)


def main():
    global client, img, img_gray, thresh,  hsv,  pub
    rate = rospy.Rate(10)  # 10Hz

    cv2.namedWindow('Source', flags=cv2.WINDOW_NORMAL)
    cv2.createTrackbar('Threshold: ', 'Source', thresh,
                       255, on_threshold_trackbar)
    cv2.createTrackbar('Gamma: ', 'Source', gamma, 50, on_gamma_callback)

    while(img is None):
        rate.sleep()

    while not rospy.is_shutdown():
        img_resize = cv2.resize(img, (img.shape[1] / 4, img.shape[0] / 4))
        img_gamma = adjust_gamma(img_resize, gamma)
        hsv = cv2.cvtColor(img_gamma, cv2.COLOR_BGR2HSV)
        img_gray = cv2.cvtColor(img_gamma, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('Source', img_gamma)

        h, s, v = cv2.split(hsv)
        h_inv = cv2.bitwise_not(h)
        ret, mask = cv2.threshold(h_inv, thresh, 255, cv2.THRESH_BINARY)

        sure_fg = cv2.bitwise_and(img_gamma, img_gamma,  mask=mask)
        img_gray = cv2.cvtColor(sure_fg, cv2.COLOR_BGR2GRAY)
        res = process()
        if res != None:
            pub.publish(res)

        key = cv2.waitKey(1) & 0xff
        if key == ord('q'):
            break
        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('vision_squid', anonymous=True)
    topic = "/top/center/image_rect_color/compressed"
    rospy.Subscriber(topic, CompressedImage, callback)
    find_squid()
