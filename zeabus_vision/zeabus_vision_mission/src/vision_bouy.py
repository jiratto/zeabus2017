#!/usr/bin/env python
"""
Copyright @ EmOne (Thailand) Co.Ltd. 2017
Author: Anol Paisal <info@emone.co.th>
Date: 2017/05/15
"""
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
import math
from std_msgs.msg import String
from zeabus_vision_srv_msg.msg import vision_msg_bouy
from zeabus_vision_srv_msg.srv import vision_srv_bouy
img = None
img_gamma = None
img_gray = None
img_resize = None
hsv = None
old_frame = None
wait = False
thresh = 184
gamma = 30
color = None


def on_gamma_callback(param):
    global gamma
    gamma = param


def adjust_gamma(image, gamma=1):
    # build a lookup table mapping the pixel values [0, 255] to
    # their adjusted gamma values
    if gamma == 0:
        g = 1.0
    else:
        g = gamma / 10.0
    invGamma = 1.0 / g
    table = np.array([((i / 255.0) ** invGamma) * 255
                      for i in np.arange(0, 256)]).astype("uint8")

# apply gamma correction using the lookup table
    return cv2.LUT(image, table)


def on_threshold_callback(param):
    global thresh
    thresh = param


def threshold_callback(params):
    global img_gray, thresh, gamma, img_gamma, color
    thresh = params
    img_gray = cv2.GaussianBlur(img_gray, (7, 7), 0)
    cv2.imshow('blur', img_gray)

    ret, thresh_out = cv2.threshold(
        img_gray, thresh, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    kernel = np.ones((3, 3), np.uint8)
    #opening = cv2.morphologyEx(img_gray, cv2.MORPH_GRADIENT, kernel, iterations = 2)
    opening = cv2.morphologyEx(
        thresh_out, cv2.MORPH_OPEN, kernel, iterations=2)

    # cv2.imshow('opening', opening)
    sure_bg = cv2.dilate(opening, kernel, iterations=5)
    # cv2.imshow('sure_bg', sure_bg)

    dist = cv2.distanceTransform(opening, cv2.DIST_L2, 3)
    cv2.normalize(dist, dist, 0, 255, cv2.NORM_MINMAX)
    dist = np.uint8(dist)

    # cv2.imshow('dist', dist)
    sure_fg = cv2.adaptiveThreshold(
        dist, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    # cv2.imshow('sure_fg', sure_fg)

    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg, sure_fg)
    cv2.imshow('unknown', unknown)

    im2, contours, hierarchy = cv2.findContours(
        unknown, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    minRect = []
    hsv_drawing = hsv.copy()
    drawing = hsv.copy()
    print 'length of contour : ' + str(len(contours))
#
    for i in range(len(contours)):
        minRect.append(cv2.minAreaRect(contours[i]))
        x, y, w, h = cv2.boundingRect(contours[i])
        cv2.rectangle(drawing, (x, y), (x + w, y + h), (0, 0, 255), 1)
        cv2.drawContours(drawing, contours, i, (0, 255, 0), 1)
        cv2.imshow('Contour', drawing)

    for i in range(len(contours)):
        box = cv2.boxPoints(minRect[i])
        box = np.int0(box)
        if box[1][1] < box[3][1] and box[0][0] < box[2][0]:
            img_roi = hsv[int(box[1][1]):int(box[3][1]),
                          int(box[0][0]):int(box[2][0])]

            img_rgb_roi = cv2.cvtColor(img_roi, cv2.COLOR_HSV2BGR)

            img_roi = cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY)
            cv2.imshow('ROI', img_roi)
            clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(3, 3))
            cl1 = clahe.apply(img_roi)
            res = np.hstack((img_roi, cl1))
            cv2.imshow('equ', res)

            circles = cv2.HoughCircles(
                img_roi, cv2.HOUGH_GRADIENT, 1.3, 100, param1=80, param2=20, minRadius=0, maxRadius=0)
            if circles != None:
                for i in circles[0, :]:

                    cv2.circle(hsv_drawing, (int(
                        box[1][0] + i[0]), int(box[1][1] + i[1])), i[2], (0, 255, 0), 2)

                    cv2.circle(
                        hsv_drawing, (int(box[1][0] + i[0]), int(box[1][1] + i[1])), 2, (0, 0, 255), 3)
                    cv2.imshow('circle', hsv_drawing)

                    w = img_rgb_roi[0, 0]
#                    print w
                    b, g, r = cv2.split(img_rgb_roi)
                    mask = np.zeros(img_rgb_roi.shape[:2], np.uint8)

                    mask[:, :] = w[0]

                    b_masked_img = cv2.subtract(b, mask)

                    b_histr, bins = np.histogram(
                        b_masked_img.ravel(), 256, [0, 256])
                    b_cdf = b_histr.cumsum()
#
                    mask[:, :] = w[1]
                    g_masked_img = cv2.subtract(g, mask)
                    g_histr, bins = np.histogram(
                        g_masked_img.ravel(), 256, [0, 256])
                    g_cdf = g_histr.cumsum()

                    mask[:, :] = w[2]
                    r_masked_img = cv2.subtract(r, mask)
                    r_histr, bins = np.histogram(
                        r_masked_img.ravel(), 256, [0, 256])
                    r_cdf = r_histr.cumsum()

                    total = (
                        b_cdf.max() - b_histr[0]) + (g_cdf.max() - g_histr[0]) + (r_cdf.max() - r_histr[0])
                    if total == 0:
                        Pb = 1.0
                        Pg = 1.0
                        Pr = 1.0
                    else:
                        Pb = (float(b_cdf.max() - b_histr[0]) / float(total))
                        Pg = (float(g_cdf.max() - g_histr[0]) / float(total))
                        Pr = (float(r_cdf.max() - r_histr[0]) / float(total))

                    if abs(Pg - Pr) < 0.2:
                        color = 'y'
                    elif Pg > Pb and Pg > Pr:
                        color = 'g'
                    elif Pr > Pb and Pr > Pg:
                        color = 'r'

                    return (int(box[1][0] + i[0]), int(box[1][1] + i[1]), int(i[2]), Pb, Pg, Pr,  color)


def callback(msg):
    global img, img_gray, hsv, gamma,  img_resize
    if wait == False:
        arr = np.fromstring(msg.data, np.uint8)
        img = cv2.imdecode(arr, 1)


def mission_callback(msg):
    global task, req
    task = msg.task.data
    req = msg.req.data
    return find_bouy()


def find_bouy():
    global client, img, img_gray, thresh, img_gamma,  hsv,  img_resize,  color
    res = None
    # cv2.namedWindow('Source', flags=cv2.WINDOW_NORMAL)
    # cv2.createTrackbar('Threshold: ', 'Source', thresh,
    #                    255, on_threshold_callback)
    # cv2.createTrackbar('Gamma: ', 'Source', gamma, 50, on_gamma_callback)
    m = vision_msg_bouy()
    while img is None:
        print 'img none'
        rospy.sleep(0.1)  # rospy.sleep(0.01)

    img_resize = cv2.resize(img, (img.shape[1] / 4, img.shape[0] / 4))
    img_gamma = adjust_gamma(img_resize, gamma)
    hsv = cv2.cvtColor(img_gamma, cv2.COLOR_BGR2HSV)
    img_gray = cv2.cvtColor(img_gamma, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('Source', img_gamma)

    h, s, v = cv2.split(hsv)
    h_inv = cv2.bitwise_not(h)
    ret, mask1 = cv2.threshold(h_inv, 162, 255, cv2.THRESH_BINARY_INV)
    mask1_inv = cv2.bitwise_not(mask1)
    ret, mask2 = cv2.threshold(h_inv, 150, 255, cv2.THRESH_BINARY)
    mask2_inv = cv2.bitwise_not(mask2)
    #cv2.imshow('mask2_inv', mask2_inv)
    mask = cv2.bitwise_or(mask1_inv,  mask2_inv)
    #cv2.imshow('mask', mask)
    img_gray = cv2.bitwise_and(img_gray, img_gray, mask=mask)
    cv2.imshow('Gray', img_gray)

    # if old_frame == None:
    #     old_frame = img_gamma.copy()
    #     pass
    # else:
    r, c = 1, 1
    x = 0
    y = 0
    radius = 0
    color_res = 'c'
    if res != None:
        r, c = img_gray.shape
        res = threshold_callback(thresh)
        x = res[0]
        y = res[1]
        color_res = res[6]
        radius = res[2]
        prob = {'y': res[3], 'g': res[4], 'r': res[5]}
        buoy = "window_x={7},window_y={8},origin_x={0},origin_y={1},radius={2},prob_blue={3},prob_green={4},prob_red={5},color={6}".format(
            x, y, radius, res[3], res[4], res[5], res[6], c, r)
        rospy.loginfo(buoy)
        # pub.publish(buoy)
    # old_frame = img_gamma.copy()
    key = cv2.waitKey(1) & 0xff

    # if key == ord('q'):
    #     break
    # rate.sleep()

    # cv2.destroyAllWindows()
    m.cx = [(x - c / 2) / c / 2]
    m.cy = [(y - r / 2) / r / 2]
    m.area = [radius]
    m.prob = [0]
    m.num = 1
    if color == color_res:
        m.appear = True
        m.prob = [prob[str(color)]]
    else:
        m.appear = False
    print m
    return m

if __name__ == '__main__':
    rospy.init_node('buoy', anonymous=True)
    topic = "/top/center/image_rect_color/compressed"
    rospy.Subscriber(topic, CompressedImage, callback)
    rospy.Service('vision_bouy', vision_srv_bouy, mission_callback)
    rospy.spin()
    # while not rospy.is_shutdown():
    #     find_bouy()
