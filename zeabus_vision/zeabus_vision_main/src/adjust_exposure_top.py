#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from scipy import stats
from sensor_msgs.msg import CompressedImage, PointCloud
from std_msgs.msg import Float32
import math
import dynamic_reconfigure.client
import time
import statistics

from matplotlib import pyplot as plt
from vision_lib import *
img = None
hsv = None
node = None
client = None
width = 480
height = 320


def callback(msg):
    global img, hsv, width, height
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (width, height))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


def set_param(param, value):
    global client
    params = {str(param): value}
    config = client.update_configuration(params)


def get_param(param):
    global node
    return rospy.get_param(str(node) + str(param), False)


def get_cv(v):
    mean = cv2.mean(v)[0]
    sd = cv2.meanStdDev(v, mean)[0]
    # print mean, sd
    return sd / mean


def inrange_ratio(min, ratio, max):
    if min <= ratio <= max:
        return True
    return False


def get_mode(v):
    try:
        MODE = statistics.mode(v)
    except ValueError:
        MODE = 127
    return MODE


def trimmed(v, trimmedValue):
    v = list(v)
    for i in range(0, trimmedValue[0]):
        v = filter(lambda a: a != i, v)

    for i in range(trimmedValue[1], 256):
        v = filter(lambda a: a != i, v)
    return v


def adjust_exposure_time():
    global hsv, img, width, height

    # cv2.namedWindow('image', flags=cv2.WINDOW_NORMAL)
    # cv2.moveWindow('image', 20, 20)
    # cv2.resizeWindow('image', width, height)
    # cv2.createTrackbar('v', 'image', 0, 255, nothing)
    while not rospy.is_shutdown():
        if hsv is None:
            rospy.sleep(0.01)
            print 'image is none'
            continue

        h, s, v = cv2.split(hsv)
        vOneD = v.ravel()
        # sOneD = s.ravel()
        vMean = cv2.mean(vOneD)[0]
        # sMean = cv2.mean(sOneD)[0]
        vMode = get_mode(vOneD)
        # sMode = get_mode(sOneD)
        vCV = get_cv(v)
        # sCV = get_cv(s)
        _, vSD = cv2.meanStdDev(vOneD, vMean)
        vSD = vSD[0]
        # _, sSD = cv2.meanStdDev(sOneD, sMean)
        # sSD = sSD[0]

        ev = get_param('exposure')

        if vMode >= 235:
            ev -= 0.1
        elif vMode <= 45:
            ev += 0.1
        max(0.5, ev)
        set_param('exposure', ev)
        # print("SD: {0} mode: {1} mean: {2}".format(vSD, vMode, vMean))
        # print("SD: {0} mode: {1} mean: {2}".format(sSD, sMode, sMean))
        # print("My Auto Exposure: {0}".format(ev))

        # cv2.imshow('image', img)
        key = cv2.waitKey(1) & 0xff
        if key == ord('q'):
            break

        rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('adjust_exposure_time_top')
    topic = '/top/center/image_rect_color/compressed'
    node = 'ueye_cam_nodelet_top_center/'
    rospy.Subscriber(topic, CompressedImage, callback)
    client = dynamic_reconfigure.client.Client(node)
    set_param('auto_exposure', False)
    set_param('auto_frame_rate', True)
    ev = 0.7
    print("Exposure Start: {0}".format(ev))
    set_param('exposure', ev)
    time.sleep(2)
    adjust_exposure_time()
