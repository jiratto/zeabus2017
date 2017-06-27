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
import statistics


frame = None
rows = None
cols = None
call = False
width = 320
height = 256
channel = 1
task = None
req = None
i = 0


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
        lower, upper = getColor('orange', 'down')
        mask = cv2.inRange(hsv, lower, upper)
    else:
        print 'request is false'
    return mask


def cam_callback(msg):
    global frame, rows, cols, width, height

    arr = np.fromstring(msg.data, np.uint8)

    frame = cv2.resize(cv2.imdecode(arr, 1), (width, height))

    rows, cols, ch = frame.shape


def mission_callback(msg):
    global task, req
    task = msg.task.data
    req = msg.req.data

    if task == 'path1' or task == 'path2':
        return do_path(msg)


def do_path(msg):

    global frame, rows, task, req, height
    images = None
    task = msg.task.data
    req = msg.req.data
    result = np.zeros((height, width))
    lower = np.array([38,5,30],np.uint8)
    upper = np.array([170,198,140],np.uint8)
    
    while frame is None or rows != height:
        print rows, height
        rospy.sleep(0.01)
    # cv2.imshow('stretcing',hsv_streching)
    # cv2.imshow('hsv',hsv)
    hsv = cv2.cvtColor(frame.copy(), cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, lower, upper)    
    h, s, v = cv2.split(hsv)
    MODE = 0
    try:
        MODE = statistics.mode(v.ravel())
    except ValueError:
        MODE = 127
    if MODE > 240:
        print 'adjust v'
        v -= 40;
    hsv_new = cv2.merge([h,s,v])
    # m = vision_msg_default()
    
    hsv_stretching = stretching(hsv_new.copy())
    rgb = cv2.cvtColor(hsv_new.copy(), cv2.COLOR_HSV2RGB)
    gray = cv2.cvtColor(rgb.copy(), cv2.COLOR_RGB2GRAY)
    # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    # cl1 = clahe.apply(gray.copy())
    blockSize = width / 3
    if blockSize % 2 == 0:
        blockSize -= 1
    # print blockSize
    th = cv2.adaptiveThreshold(
        gray.copy(), 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, blockSize, 1)
   
    mask = ~mask
    res = th & mask
    _, contours, hierarchy = cv2.findContours(
        res.copy(), cv2.RETR_TREE , cv2.CHAIN_APPROX_NONE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 200 and find_shape(cnt,4):
            cv2.drawContours(result, [cnt], 0, (255,255,255), 1)

    cv2.imshow('gray',gray)
    cv2.imshow('hsv', hsv)
    cv2.imshow('hsv_new', hsv_new)
    cv2.imshow('hsv_stretching', hsv_stretching)
    cv2.imshow('rgb',rgb)
    cv2.imshow('th', th)
    cv2.imshow('res',res)
    cv2.imshow('result',result)

   
    k = cv2.waitKey(1) & 0xff
    if k == ord('q'):
        rospy.signal_shutdown('')

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
    rospy.Service('vision2', vision_srv_default(), mission_callback)

    rospy.spin()
