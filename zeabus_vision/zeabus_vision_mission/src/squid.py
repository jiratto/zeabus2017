#!/usr/bin/env python
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
width = int(1152 / 3)
height = int(870 / 3)


def img_callback(msg):
    global img, width, height
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (width, height))


def mission_callback(msg):
    print('mission_callback')
    req = msg.req.data
    print('request: ') + str(req)
    return find_squid(req)


def find_squid(req):
    global img, width, height

    lowerY, upperY = getColor('yellow', 'top')
    lowerW, upperW = getColor('white', 'top')
    lowerR, upperR = getColor('red', 'top')

    font = cv2.FONT_HERSHEY_SIMPLEX

    kernelFrame = get_kernal('plus', (5, 5))
    kernelRow = get_kernal('rect', (3, 1))
    kernelCol = get_kernal('rect', (1, 3))

    m = vision_msg_default()
    m.appear = False
    m.angle = 0
    m.x = 0
    m.y = 0
    minArea = 500
    resWhite = np.zeros((height, width))

    while img is None:
        print('img is none in loop')

    resPreprocess = preprocess_squid(img)
    resHSV = cv2.cvtColor(resPreprocess.copy(), cv2.COLOR_BGR2HSV)
    resImg = resPreprocess.copy()

    resultFourCir = []
    resultTwoCir = []

    maskW = cv2.inRange(resHSV, lowerW, upperW)
    maskR = cv2.inRange(resHSV, lowerR, upperR)

    maskYPre = cv2.inRange(resHSV, lowerY, upperY)
    maskY = dilate(maskYPre, kernelFrame)
    maskYBlur = cv2.medianBlur(maskYPre, 3)

    mask1 = cv2.subtract(maskYBlur, maskW)
    mask = cv2.subtract(mask1, maskR)
    maskInv = np.invert(mask)

    _, th = cv2.threshold(maskInv, 10, 255, 0)
    _, contours, _ = cv2.findContours(
        th.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    statusFilter = False

    for c in contours:
        area = cv2.contourArea(c)
        # cv2.drawContours(resImg, [c], -1, (222, 2, 222), 3)
        if area < minArea:
            continue
        (x, y), r = cv2.minEnclosingCircle(c)
        areaCir = (math.pi * (r**2))
        if area / areaCir <= 0.65:
            continue
        if statusFilter and (x - x_before)**2 + (y - y_before)**2 <= 10**2:
            continue
        cv2.circle(resImg, (int(x), int(y)), int(r), (255, 255, 0), 3)
        cv2.circle(resImg, (int(x), int(y)), 2, (0, 255, 0), -1)
        cv2.putText(resImg, 'Area: %.2f %d' %
                    (((areaCir / (width * height))), (areaCir)),
                    (int(x), int(y)), font, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        resultFourCir.append([x, y, areaCir / (width * height)])
        x_before = x
        y_before = y
        statusFilter = True

    if len(resultFourCir) > 0:
        resultFourCir = sorted(resultFourCir, key=lambda l: l[0], reverse=True)
        cutterX = (resultFourCir[0][0] + resultFourCir[-1][0]) / 2.0
        cv2.line(resImg, (int(cutterX - 10), 0),
                 (int(cutterX - 10), height), (212, 123, 132), 5)
        for res in resultFourCir:
            x = res[0]
            # < x_before and abs(y - y_before) <= 20:
            if x <= cutterX - 10:  # statusFilter and
                continue
            resultTwoCir.append(res)

    if len(resultTwoCir) > 0:
        resultTwoCir = sorted(resultTwoCir, key=lambda l: l[2], reverse=True)
        m.appear = True
        if req == 'a':
            x = 0
            ct = 0
            for res in resultTwoCir:
                x += resultTwoCir[0]
                ct += 1
            m.x = int(x / (ct * 1.0))
            m.y = int(height / 2.0)
        elif req == 's':
            m.x = resultTwoCir[-1][0]
            m.y = resultTwoCir[-1][1]
            m.area = resultTwoCir[-1][2]
        elif req == 'b':
            m.x = resultTwoCir[0][0]
            m.y = resultTwoCir[0][1]
            m.area = resultTwoCir[0][2]
        offsetW = width / 2.0
        offsetH = height / 2.0

        cv2.circle(resImg, ((int(m.x)), int(m.y)), 6, (255, 255, 255), -1)

        m.x = -((m.x - offsetW) / offsetW)
        m.y = (offsetH - m.y) / offsetH

    else:
        m.appear = False

    print m

    publish_result(mask, 'gray', '/result_squid_range')
    publish_result(th, 'gray', '/result_squid_range_inv')
    publish_result(resImg, 'bgr', '/result_squid')
    return m

if __name__ == '__main__':
    rospy.init_node('vision_squid', anonymous=True)
    topic = "/top/center/image_rect_color/compressed"
    rospy.Subscriber(topic, CompressedImage, img_callback)
    rospy.Service('vision_squid', vision_srv_default(), mission_callback)
    rospy.spin()
