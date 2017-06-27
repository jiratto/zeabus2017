#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from scipy import stats
from sensor_msgs.msg import CompressedImage
import math
import dynamic_reconfigure.client
import time
import statistics
from matplotlib import pyplot as plt
img = None
hsv = None
node = None
client = None


def callback(msg):
    global img, hsv
    width = 160
    height = 128
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


def get_cv():
    global hsv
    h, s, v = cv2.split(hsv)
    mean = cv2.mean(v)
    sd = cv2.meanStdDev(v, mean)
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
    global hsv, img

    while not rospy.is_shutdown():
        while img is None:
            rospy.sleep(0.01)
            key = cv2.waitKey(1) & 0xff
            if key == ord('q'):
                break

        key = cv2.waitKey(1) & 0xff
        # print('======= AE Bottom =======')
        h, s, v = cv2.split(hsv)
        vOneD = v.ravel()
        mean = cv2.mean(vOneD)[0]

        MODE = get_mode(vOneD)
        _, SD = cv2.meanStdDev(vOneD, mean)
        SD = SD[0]
        CV = SD / mean
        ev = get_param('exposure')
        # ev = max(ev, 0.3)

        # MODE = get_mode(vOneD)
        if SD < 70:
            if MODE >= 240:
                ev -= 0.05
                set_param('exposure', ev)
            elif MODE <= 60:
                ev += 0.05
                set_param('exposure', ev)

        print("SD: {0} mode: {1} mean: {2}".format(SD, MODE, mean))
        print("CV: {0} AE: {1}".format(CV, ev))

        # print('========================')
        cv2.imshow('v', v)
        if key == ord('q'):
            break
        # plt.plot(CV, 'r', MODE, 'g', mean, 'y', SD)
        # plt.show()
        rospy.sleep(0.25)
if __name__ == '__main__':
    rospy.init_node('adjust_exposure_time_top')
    topic = rospy.get_param('auto_exposure/cameraTopic',
                            '/top/center/image_rect_color/compressed')
    node = rospy.get_param('auto_exposure/cameraManager',
                           'ueye_cam_nodelet_top_center/')

    rospy.Subscriber(topic, CompressedImage, callback)
    client = dynamic_reconfigure.client.Client(node)
    set_param('auto_exposure', False)
    set_param('auto_frame_rate', True)
    ev = 1
    print("Exposure Start: {0}".format(ev))
    set_param('exposure', ev)
    time.sleep(2)
    adjust_exposure_time()
