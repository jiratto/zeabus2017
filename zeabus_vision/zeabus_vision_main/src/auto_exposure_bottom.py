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

hsvL = None
hsvR = None
nodeL = None
nodeR = None
clientL = None
clientR = None
width = int(1280/2)
height = int(768/2)


def imageL_callback(msg):
    global hsvL, width, height
    arr = np.fromstring(msg.data, np.uint8)
    imgL = cv2.resize(cv2.imdecode(arr, 1), (width, height))
    hsvL = cv2.cvtColor(imgL, cv2.COLOR_BGR2HSV)


def imageR_callback(msg):
    global hsvR, width, height
    arr = np.fromstring(msg.data, np.uint8)
    imgR = cv2.resize(cv2.imdecode(arr, 1), (width, height))
    hsvR = cv2.cvtColor(imgR, cv2.COLOR_BGR2HSV)


def set_param(camera, param, value):
    global clientL, clientR
    params = {str(param): value}
    if camera == 'L':
        config = clientL.update_configuration(params)
    else:
        config = clientR.update_configuration(params)


def get_param(camera, param):
    global nodeL, nodeR
    if camera == 'L':
        return rospy.get_param(str(nodeL) + str(param), False)
    else:
        return rospy.get_param(str(nodeR) + str(param), False)


def get_cv(v):
    mean = cv2.mean(v)[0]
    sd = cv2.meanStdDev(v, mean)[0]
    # print mean, sd
    return sd / mean


def inrange_ratio(min, ratio, max):
    if min <= ratio <= max:
        return True
    return False


def trimmed(v, trimmedValue):
    v = list(v)
    for i in range(0, trimmedValue[0]):
        v = filter(lambda a: a != i, v)

    for i in range(trimmedValue[1], 256):
        v = filter(lambda a: a != i, v)
    return v


def adjust_exposure_time():
    global hsvL, hsvR, width, height

    while not rospy.is_shutdown():
        if hsvL is None:
            print 'image is none'
            continue

        h, s, v = cv2.split(hsvL)
        vOneD = v.ravel()
        vMean = cv2.mean(vOneD)[0]
        vMode = get_mode(vOneD)
        vCV = get_cv(v)
        _, vSD = cv2.meanStdDev(vOneD, vMean)
        vSD = vSD[0]

        ev = get_param('L', 'exposure')
        if vMode >= 235:
            ev -= 0.1
        elif 50 <= vMode <= 100:
            ev += 0.05
        elif vMode <= 45:
            ev += 0.1
        max(0.5, ev)
        set_param('L', 'exposure', ev)
        key = cv2.waitKey(1) & 0xff
        if key == ord('q'):
            break

if __name__ == '__main__':
    cameraPos = rospy.get_param('cameraPos', 'bottom')
    nodeName = 'Auto_Exposure_' + cameraPos
    topicL = rospy.get_param('cameraTopicLeft','/bottom/left/image_raw/compressed')
    print topicL
    # topicR = str(rospy.get_param('cameraTopicRight'))
    nodeL = rospy.get_param('cameraNodeLeft', 'ueye_cam_nodelet_bottom_left/')
    # nodeR = str(rospy.get_param('cameraNodeRight'))
    rospy.init_node(nodeName)
    rospy.Subscriber(topicL, CompressedImage, imageL_callback)
    # rospy.Subscriber(topicR, CompressedImage, imageR_callback)
    clientL = dynamic_reconfigure.client.Client(nodeL)
    # clientR = dynamic_reconfigure.client.Client(nodeR)
    set_param('L', 'auto_exposure', False)
    set_param('L', 'auto_frame_rate', True)
    ev = 0.7
    print("Exposure Start: {0}".format(ev))
    set_param('L', 'exposure', ev)
    time.sleep(2)
    adjust_exposure_time()
