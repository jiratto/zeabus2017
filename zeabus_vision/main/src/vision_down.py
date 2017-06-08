#!/usr/bin/env python
import cv2
import numpy as np
from std_msgs.msg import String
from zeabus_vision_srv_msg.srv import vision_srv_default
from zeabus_vision_srv_msg.msg import vision_msg_default
from sensor_msgs.msg import CompressedImage
import rospy
from operator import itemgetter
from vision_lib import *

frame = None
rows = None
cols = None
call = False
width = 320
height = 256
channel = 1
task = None
req = None


def not_found(m):
    m.x = 0
    m.y = 0
    m.area = -999
    m.appear = False


def getMask():
    global frame, task, req
    mask = None
    hsv = process_img_down(frame)
    if task == 'path1':
        lower, upper = getColor('orange','down')
        mask = cv2.inRange(hsv, lower, upper)
    else:
        print 'request is false'
    return mask


def cam_callback(msg):
    global frame, rows, cols, width, height

    arr = np.fromstring(msg.data, np.uint8)
    frame = cv2.imdecode(arr, 1)
    frame = cv2.resize(frame, (width, height))
    rows, cols, ch = frame.shape


def mission_callback(msg):
    global task, req
    task = msg.task.data
    req = msg.req.data

    if task == 'path1' or task == 'path2':
        return do_path(msg)


def do_path(msg):
    global frame, rows, task, req

    task = msg.task.data
    req = msg.req.data

    while frame is None or rows != height:
        rospy.sleep(0.01)

    m = vision_msg_default()
    mask = getMask()
    h, w = mask.shape

    erode = cv2.erode(mask, np.array([[1] * 3] * 3))
    dilate = cv2.dilate(erode, np.array([[1] * 3] * 3))
    _, contours, hierarchy = cv2.findContours(
        dilate.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    result = dilate.copy()
    result = cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)
    sq_found = []

    for c in contours:
        M = cv2.moments(c)
        rect = (x, y), (ww, hh), angle = cv2.minAreaRect(c)
        area = ww * hh

        if area <= 0 or ww <= 0:
            continue
        real_area = cv2.contourArea(c)
        ratio_area = real_area / area
        ratio_scale = hh / ww

        box = cv2.boxPoints(rect)
        box = np.int0(box)
    #	print real_area, ratio_area, ratio_scale
        err = 3
        if (ratio_area > 0.6 and real_area >= 200 and (ratio_scale > 3 or ratio_scale < 0.33) and 
        (y - hh / 2 > err + h and y + hh / 2 < h - err and x - ww / 2 > err and x + ww / 2 < w - err or real_area >= 200)):
            sq_found.append(
                [real_area, (x, y), 90 - Oreintation(M)[0] * 180 / math.pi])
            cv2.drawContours(result, [box], 0, (0, 255, 0), 1, 8)

    if len(sq_found) == 0:
        not_found(m)
    else:
        sq_found = sorted(sq_found, key=itemgetter(0))
        x, y = sq_found[-1][1]
        cv2.circle(result, (int(x), int(y)), 5, (0, 0, 255), -1)

        m.x = ((height / 2.0) - y) / (height / 2.0)
        m.y = ((width / 2.0) - x) / (width / 2.0)
        lx, ly = m.x, m.y
        m.angle = sq_found[-1][2]
        m.area = sq_found[-1][0] / (width * height)
        m.appear = True

    cv2.imshow('result', result)
    k = cv2.waitKey(1) & 0xff
    if k == ord('q'):
        rospy.signal_shutdown('')
    # print m
    return m

if __name__ == "__main__":
    rospy.init_node('vision_down')
    rospy.Subscriber('/rightcam_bottom/image_raw/compressed',
                     CompressedImage, cam_callback)
    # rospy.Subscriber('/leftcam_bottom/image_raw/compressed',CompressedImage,cam_callback)

    while not rospy.is_shutdown():
        msg = vision_srv_default()
        msg.task = String('path1')
        msg.req = String('orange')
        do_path(msg)
        # msg.task = String('navigate')
        # msg.req = String('yellow')
        # do_navigate(msg)
        # msg.task = String('pipe')
        # msg.req = String('red')
        # do_pipe(msg)
    rospy.Service('vision2', vision_srv, mission_callback)
    rospy.spin()
