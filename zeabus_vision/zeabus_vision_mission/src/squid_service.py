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

    kernelFrame = get_kernal('plus', (3, 3))
    kernelRow = get_kernal('rect', (5, 3))
    kernelCol = get_kernal('rect', (3, 5))

    m = vision_msg_default()
    m.appear = False
    m.angle = 0
    m.x = 0
    m.y = 0

    resWhite = np.zeros((height, width))

    while img is None:
        print('img is none in loop')

    resPreprocess = preprocess_squid(img)
    resHSV = cv2.cvtColor(resPreprocess.copy(), cv2.COLOR_BGR2HSV)
    resImg = resPreprocess.copy()

    result = []

    # resY = cv2.inRange(resHSV, lowerY, upperY)
    maskY = cv2.inRange(resHSV, lowerY, upperY)
    # resDilateRow = dilate(resY, kernelRow)
    # resDilateCol = dilate(resY, kernelCol)
    # resDilate = resDilateRow + resDilateCol
    # resErode = erode(resDilate, kernelFrame)
    # maskY = erode(resErode, kernelFrame)
    # maskY = dilate(resY, kernelFrame)

    # resW = cv2.inRange(resHSV, lowerW, upperW)
    # resDilateRow = dilate(resW, kernelRow)
    # resDilateCol = dilate(resW, kernelCol)
    # resDilate = resDilateRow + resDilateCol
    # resErode = erode(resDilate, kernelFrame)
    # maskW = erode(resDilate, kernelFrame)
    maskW = cv2.inRange(resHSV, lowerW, upperW)
    maskR = cv2.inRange(resHSV, lowerR, upperR)

    mask1 = cv2.subtract(maskY, maskW)
    mask = cv2.subtract(mask1, maskR)
    # _, contours, _ = cv2.findContours(
    #     mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # for c in contours:
    #     M = cv2.moments(c)
    #     area = cv2.contourArea(c)
    #     if area < 100:
    #         continue
    #     if cut_contours(M, width, height, 25, 25):
    #         continue
    #     const = 0.04
    #     peri = const * cv2.arcLength(c, True)
    #     approx = cv2.approxPolyDP(c, const * peri, True)

    #     approxLen = len(approx)
    #     if approxLen >= 8:
    #         rect = cv2.minAreaRect(c)
    #         (x, y), (w, h), angle = rect
    #         areaRect = int(w * h)
    #         ratioScale = w * 1.0 / h
    #         if areaRect >= 700 and 0.85 <= ratioScale <= 1.15:
    #             areaScale = (areaRect * 1.0) / (width * height)
    #             cv2.rectangle(resImg, (int(x - w / 2), int(y - h / 2)),
    #                           (int(x + w / 2), int(y + h / 2)), (0, 255, 0), 2)
    #             cv2.putText(resImg, str(areaScale) + ' ' + str(approxLen), (int(x - w / 2), int(y - h / 2)), font, 0.5,
    #                         (0, 255, 255), 1, cv2.LINE_AA)
    #             cv2.circle(resImg, ((int(x)),
    #                                 int(y)), 4, (255, 255, 0), -1)
    #             result.append([x, y, areaScale])
    # circles = cv2.HoughCircles(mask.copy(), cv2.HOUGH_GRADIENT, dp=1,
    # minDist=60, param1=50, param2=20, minRadius=10, maxRadius=100)
    circles = cv2.HoughCircles(mask.copy(), cv2.HOUGH_GRADIENT, dp=1,
                               minDist=60, param1=40, param2=30, minRadius=10, maxRadius=150)
    if not circles is None:
        print circles[0]
        circles = sorted(circles[0], key=lambda l: l[2])
        for circle in circles:
            print circle
            [x, y, r] = circle
            if r > 100:
                continue
            area = (math.pi * (r**2)) / (width * height * 1.0)
            cv2.circle(resImg, (x, y), r, (255, 255, 0), 3)
            cv2.circle(resImg, (x, y), 2, (0, 255, 0), -1)
            cv2.putText(resImg, 'R: ' + str(r) + 'Area: ' + str(area), (x, y), font, 0.25,
                        (0, 255, 255), 1, cv2.LINE_AA)
            result.append([x, y, area])
    # if not circles1 is None:
    #     print circles1[0]
    #     circles1 = sorted(circles1[0], key=lambda l: l[2])
    #     for circle in circles1:
    #         print circle
    #         [x, y, r] = circle
    #         if r > 100:
    #             continue
    #         area = (math.pi * (r**2)) / (width * height * 1.0)
    #         cv2.circle(resImg, (x, y), r, (0, 255, 0), 5)
    #         cv2.circle(resImg, (x, y), 2, (0, 255, 0), -1)
    #         cv2.putText(resImg, str(r), (x, y), font, 0.5,
    #                     (0, 255, 0), 1, cv2.LINE_AA)
            # result.append([x, y, area])
    if len(result) > 0:
        result = sorted(result, key=lambda l: l[2], reverse=True)
        m.appear = True
        if req == 'a':
            x = 0
            ct = 0
            for res in result:
                x += res[0]
                ct += 1
            m.x = int(x / (ct * 1.0))
            m.y = int(height / 2.0)
        elif req == 's':
            m.x = result[-1][0]
            m.y = result[-1][1]
            m.area = result[-1][2]
        elif req == 'b':
            m.x = result[0][0]
            m.y = result[0][1]
            m.area = result[0][2]
        offsetW = width / 2.0
        offsetH = height / 2.0

        cv2.circle(resImg, ((int(m.x)), int(m.y)), 5, (255, 255, 255), -1)

        m.x = -((m.x - offsetW) / offsetW)
        m.y = (offsetH - m.y) / offsetH

    else:
        m.appear = False

    print m

    publish_result(mask, 'gray', '/result_squid_range')
    publish_result(resImg, 'bgr', '/result_squid')
    return m

if __name__ == '__main__':
    rospy.init_node('vision_squid', anonymous=True)
    topic = "/top/center/image_rect_color/compressed"
    rospy.Subscriber(topic, CompressedImage, img_callback)
    # find_squid()
    rospy.Service('vision_squid', vision_srv_default(), mission_callback)
    rospy.spin()
