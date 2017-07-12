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
    return find_bouy(req)


def find_circle(imgBIN, resImg):
    global width, height
    result = []
    font = cv2.FONT_HERSHEY_SIMPLEX
    # circles = cv2.HoughCircles(imgBIN.copy(), cv2.HOUGH_GRADIENT, dp=1,
    # minDist=60, param1=40, param2=30, minRadius=10, maxRadius=150)

    # if not circles is None:
    #     print circles[0]
    #     circles = sorted(circles[0], key=lambda l: l[2])
    #     for circle in circles:
    #         print circle
    #         [x, y, r] = circle
    #         if r > 100:
    #             continue
    #         area = (math.pi * (r**2)) / (width * height * 1.0)
    #         cv2.circle(resImg, (x, y), r, (255, 255, 0), 3)
    #         cv2.circle(resImg, (x, y), 2, (0, 255, 0), -1)
    #         cv2.putText(resImg, 'R: ' + str(r) + 'Area: ' + str(area), (x, y), font, 0.25,
    #                     (0, 255, 255), 1, cv2.LINE_AA)
    #         result.append([x, y, area])
    # result = sorted(result, key=lambda l: l[2], reverse=True)
    # return result, resImg
    # _, contours, _ = cv2.findContours(
    #     imgBIN.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # for c in contours:
    #     M = cv2.moments(c)
    #     area = cv2.contourArea(c)
    #     if area < 100:
    #         continue
    #     # if cut_contours(M, width, height, 50, 50):
    #     #     continue
    #     const = 0.04
    #     peri = const * cv2.arcLength(c, True)
    #     print cv2.arcLength(c, True)
    # approxPolyDP(c,  const * peri, True)
    #     approxLen = len(approx)
    #     if approxLen >= 8:
    #         rect = cv2.minAreaRect(c)
    #         (x, y), (w, h), angle = rect
    #         areaRect = int(w * h)
    #         ratioScale = w * 1.0 / h
    #         if areaRect >= 200 and 0.8 <= ratioScale <= 1.2:
    #             areaScale = (areaRect * 1.0) / (width * height)
    #             cv2.rectangle(resImg, (int(x - w / 2), int(y - h / 2)),
    #                           (int(x + w / 2), int(y + h / 2)), (0, 0, 0), 2)
    #             cv2.putText(resImg, str(area) + ' ' + str(areaScale) + ' ' + str(approxLen), (int(x - w / 2), int(y - h / 2)), font, 0.5,
    #                         (0, 255, 255), 1, cv2.LINE_AA)
    #             cv2.circle(resImg, ((int(x)),
    #                                 int(y)), 4, (255, 0, 0), -1)
    #             resultY.append([x, y, areaScale])
    #     cv2.drawContours(resImg, [c], 0, (255, 255, 255), 2)


def process_mask(imgBIN):
    kernelFrame = get_kernal('plus', (3, 3))

    kernelRow = get_kernal('rect', (5, 3))
    kernelCol = get_kernal('rect', (3, 5))

    resDilateRow = dilate(imgBIN, kernelRow)
    resDilateCol = dilate(imgBIN, kernelCol)
    resDilate = resDilateRow + resDilateCol
    resErode1 = erode(resDilate, kernelFrame)
    resErode = erode(resErode1, kernelFrame)
    return resErode


def find_bouy(req):
    global img, width, height
    m = vision_msg_bouy()
    lowerY, upperY = getColor('yellow', 'top')
    lowerR, upperR = getColor('red', 'top')
    minArea = 30
    font = cv2.FONT_HERSHEY_SIMPLEX

    while img is None:
        print('img is none in loop')
        continue
    mode = 2
    resPreprocess = preprocess_bouy(img)
    resHSV = cv2.cvtColor(resPreprocess.copy(), cv2.COLOR_BGR2HSV)
    resImg = resPreprocess.copy()

    resY = cv2.inRange(resHSV, lowerY, upperY)
    processMaskY = process_mask(resY)
    resR = cv2.inRange(resHSV, lowerR, upperR)
    processMaskR = process_mask(resR)
    mask = cv2.bitwise_or(processMaskR, processMaskR, mask=processMaskY)
    maskInv = np.invert(mask)
    maskCrop = crop_gray(maskInv, 70, 40)
    kernelFrame = get_kernal('plus', (5, 5))
    maskErode = erode(maskCrop, kernelFrame)
    _, th = cv2.threshold(maskErode, 10, 255, 0)
    _, contours, _ = cv2.findContours(
        th.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    statusFilter = False
    resultCir = []
    for c in contours:
        area = cv2.contourArea(c)
        cv2.drawContours(resImg, [c], -1, (222, 2, 222), 3)
        if area < minArea:
            continue
        (x, y), r = cv2.minEnclosingCircle(c)
        areaCir = (math.pi * (r**2))
        if area / areaCir <= 0.35:
            continue
        if statusFilter and (x - x_before)**2 + (y - y_before)**2 <= 10**2:
            continue
        cv2.circle(resImg, (int(x), int(y)), int(r), (255, 255, 255), 3)

        cv2.putText(resImg, 'Area: %.2f %d' %
                    (((areaCir / (width * height))), (areaCir)),
                    (int(x), int(y)), font, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        resultCir.append([x, y, areaCir / (width * height)])
        x_before = x
        y_before = y
        statusFilter = True

    resultCir = sorted(resultCir, key=lambda l: l[0], reverse=False)
    resultR = []
    resultG = []
    resultY = []
    # mode = 1  r y g
    # mode = 2 g y r
    for i in xrange(min(3, len(resultCir))):
        x, y, area = resultCir[i]
        if i == 0:
            if mode == 1:
                resultR = [[x, y, area]]
                color = (0, 0, 255)
            else:
                resultG = [[x, y, area]]
                color = (0, 255, 0)
            cv2.circle(resImg, (int(x), int(y)), 6, color, -1)
        elif i == 1:
            resultY = [[x, y, area]]
            color = (255, 0, 0)
            cv2.circle(resImg, (int(x), int(y)), 6, color, -1)
        else:
            if mode == 2:
                resultR = [[x, y, area]]
                color = (0, 0, 255)
            else:
                resultG = [[x, y, area]]
                color = (0, 255, 0)
            cv2.circle(resImg, (int(x), int(y)), 6, color, -1)

    # resultR, resImg = find_circle(maskR, resImg)
    # resultG, resImg = find_circle(maskG, resImg)
    # resultY, resImg = find_circle(maskY, resImg)
    cx = 0
    cy = 0
    if len(resultR) > 0:
        m.num = 1
        cx = resultR[0][0]
        cy = resultR[0][1]
        m.area = [resultR[0][2]]
        m.color = [0]
        m.appear = True
    elif len(resultG) > 0:
        m.num = 1
        cx = resultG[0][0]
        cy = resultG[0][1]
        m.area = [resultG[0][2]]
        m.color = [0]
        m.appear = True
    elif len(resultY) > 0:
        m.num = 1
        cx = resultY[0][0]
        cy = resultY[0][1]
        m.area = [resultY[0][2]]
        m.color = [0]
        m.appear = True
    else:
        not_found()

    # if req == 'r':
    #     if len(resultR) > 0:
    #         m.num = 1
    #         cx = resultR[0][0]
    #         cy = resultR[0][1]
    #         m.area = [resultR[0][2]]
    #         m.color[0]
    #         m.appear = True
    #     else:
    #         not_found()
    # elif req == 'g':
    #     if len(resultG) > 0:
    #         m.num = 1
    #         cx = resultG[0][0]
    #         cy = resultG[0][1]
    #         m.area = [resultG[0][2]]
    #         m.color[0]
    #         m.appear = True
    #     else:
    #         not_found()
    # elif req == 'y':
    #     if len(resultY) > 0:
    #         m.num = 1
    #         cx = resultY[0][0]
    #         cy = resultY[0][1]
    #         m.area = [resultY[0][2]]
    #         m.color = [0]
    #         m.appear = True
    #     else:
    #         not_found()
    # elif req == 'a':
    #     ct = 0
    #     if len(resultY) > 0:
    #         ct += 1
    #     if len(resultR) > 0:
    #         ct += 1
    #     if len(resultG) > 0:
    #         ct += 1
    #     if ct > 0:
    #         m.num = ct
    #         m.cx = [0]
    #         m.cy = [0]
    #         m.area = [0]
    #         m.color = [0]
    #         m.appear = True
    #         return m
    #     else:
    #         not_found()
    # else:
    #     not_found()
    cv2.circle(resImg, ((int(cx)),
                        int(cy)), 5, (0, 0, 0), -1)
    offsetW = width / 2.0
    offsetH = height / 2.0
    cx = -((cx - offsetW) / offsetW)
    cy = (offsetH - cy) / offsetH
    m.cx = [cx]
    m.cy = [cy]

    publish_result(resImg, 'bgr', '/result_img')
    publish_result(mask, 'gray', '/mask')
    publish_result(th, 'gray', '/mask_inv')
    # publish_result(maskG, 'gray', '/mask_g')
    # publish_result(maskR, 'gray', '/mask_r')

    print m
    return m


def not_found():
    m = vision_msg_bouy()
    m.num = 0
    m.cx = [0]
    m.cy = [0]
    m.area = [0]
    m.color = [0]
    m.appear = False
    return m
if __name__ == '__main__':
    rospy.init_node('vision_squid', anonymous=True)
    topic = "/top/center/image_rect_color/compressed"
    rospy.Subscriber(topic, CompressedImage, img_callback)
    # find_bouy()
    rospy.Service('vision_bouy', vision_srv_bouy(), mission_callback)
    rospy.spin()
