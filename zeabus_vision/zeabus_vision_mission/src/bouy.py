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
resultMemory = []
getMemoryStatus = True
xMemory = 0
yMemory = 0


def image_callback(msg):
    global img, width, height
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (width, height))


def mission_callback(msg):
    print('mission_callback')
    req = msg.req.data
    print('request: ') + str(req)
    return find_bouy(req)


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
    global img, width, height, resultMemory, getMemoryStatus, xMemory, yMemory
    m = vision_msg_bouy()
    lowerY, upperY = get_color('yellow', 'top', 'bouy')
    lowerR, upperR = get_color('red', 'top', 'bouy')
    lowerYY, upperYY = get_color('orange', 'top', 'bouy')
    minArea = 100
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
    resYY = cv2.inRange(resHSV, lowerYY, upperYY)
    processMaskYY = process_mask(resYY)

    mask = cv2.bitwise_or(processMaskR, processMaskR, mask=processMaskY)
    maskInv = np.invert(mask)
    maskCrop = crop_gray(maskInv, 60, 30)
    kernelFrame = get_kernal('plus', (5, 5))
    maskErode = erode(maskCrop, kernelFrame)
    _, th = cv2.threshold(maskErode, 20, 255, 0)
    _, contours, _ = cv2.findContours(
        th.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    cx = 0
    cy = 0
    # mode = 1  r y g
    # mode = 2 g y r
    if req == 'a':
        statusFilter = False
        resultCir = []
        ct = 0
        for c in contours:
            area = cv2.contourArea(c)
            cv2.drawContours(resImg, [c], -1, (222, 2, 222), 3)
            if area < minArea:
                continue
            (x, y), r = cv2.minEnclosingCircle(c)
            areaCir = (math.pi * (r**2))
	# edit_radius all
            if area / areaCir <= 0.40:
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
            ct += 1
        m.cx = [0]
        m.cy = [0]
        m.area = [0]
        m.color = [0]
        if ct == 3:
            resultMemory.append(resultR[0])
            resultMemory.append(resultY[0])
            resultMemory.append(resultG[0])
            m.num = ct
            m.appear = True
        else:
            m.num = ct
            m.appear = False
    elif req == 'o':
        _, th = cv2.threshold(processMaskYY, 20, 255, 0)
        _, contours, _ = cv2.findContours(
            th.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        statusFilter = False
        resultCir = []
        ct = 0
        for c in contours:
            area = cv2.contourArea(c)
            cv2.drawContours(resImg, [c], -1, (222, 2, 222), 3)
            if area < minArea:
                continue
            (x, y), r = cv2.minEnclosingCircle(c)
            areaCir = (math.pi * (r**2))
            if area / areaCir <= 0.50:
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
        if len(resultCir) > 0:
            resultCir = sorted(resultCir, key=lambda l: l[2], reverse=True)
            resultCir = resultCir[0]
            m.num = 1
            m.cx = [resultCir[0]]
            m.cy = [resultCir[1]]
            m.area = [resultCir[2]]
            m.color = [0]
            m.appear = True
            # return m
        else:
            m.num = 0
            m.cx = [0]
            m.cy = [0]
            m.area = [0]
            m.color = [0]
            m.appear = False
            return m
    else:
        if getMemoryStatus:
            if req == 'r':
                resultMemory = resultMemory[0]
            elif req == 'y':
                resultMemory = resultMemory[1]
            elif req == 'g':
                resultMemory = resultMemory[2]
            xMemory = resultMemory[0]
            yMemory = resultMemory[1]

            getMemoryStatus = False

        resultCir = []

        for c in contours:
            area = cv2.contourArea(c)
            cv2.drawContours(resImg, [c], -1, (222, 2, 222), 3)
            if area < minArea:
                continue
            (x, y), r = cv2.minEnclosingCircle(c)
            areaCir = (math.pi * (r**2))
	# each bouy radius
            if area / areaCir <= 0.30:
                continue
            if (x - xMemory)**2 + (y - yMemory)**2 >= 15**2:
                continue
            cv2.circle(resImg, (int(x), int(y)), int(r), (255, 255, 255), 3)

            cv2.putText(resImg, 'Area: %.2f %d' %
                        (((areaCir / (width * height))), (areaCir)),
                        (int(x), int(y)), font, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
            resultCir.append([x, y, areaCir / (width * height)])

        if len(resultCir) > 0:
            resultCir = sorted(resultCir, key=lambda l: l[2], reverse=False)

            m.num = 1
            cx = resultCir[-1][0]
            cy = resultCir[-1][1]
            xMemory = cx
            yMemory = cy
            m.area = [resultCir[-1][2]]
            m.color = [0]
            m.appear = True
        else:
            not_found()
    cv2.circle(resImg, ((int(cx)),
                        int(cy)), 5, (0, 0, 0), -1)
    offsetW = width / 2.0
    offsetH = height / 2.0
    cx = -((cx - offsetW) / offsetW)
    cy = (offsetH - cy) / offsetH
    m.cx = [cx]
    m.cy = [cy]

    publish_result(resImg, 'bgr', '/bouy_result')
    publish_result(mask, 'gray', '/bouy_mask')
    publish_result(th, 'gray', '/bouy_mask_inv')

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
    rospy.Subscriber(topic, CompressedImage, image_callback)
    # find_bouy()
    rospy.Service('vision_bouy', vision_srv_bouy(), mission_callback)
    rospy.spin()
