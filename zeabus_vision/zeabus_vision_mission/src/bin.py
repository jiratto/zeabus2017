#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
import math
import sys
sys.path.append ('/home/zeabus/catkin_ws/src/src_code/zeabus_vision/main/src/')
from vision_lib import *
from sensor_msgs.msg import CompressedImage
from zeabus_vision_srv_msg.msg import vision_msg_default
from zeabus_vision_srv_msg.srv import vision_srv_default

img = None
height = None
width = None

def adjust_gamma(image, gamma=1.0):
    invGamma = 1.0/gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
    for i in np.arange(0, 256)]).astype("uint8")

    return cv2.LUT(image, table)

def canny(image, threshold1, threshold2):
    return cv2.Canny(image, threshold1, threshold2)

def find_bin():
    global img

    lowerOrange, upperOrange = getColor('orange', 'down')

    while not rospy.is_shutdown():
        while img is None:
            print('None')
        image = img.copy()
        imageForDraw = img.copy()
        image = adjust_gamma(image)
        hsv = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2HSV)
        orangeImage = cv2.inRange(hsv, lowerOrange, upperOrange)
        
        cv2.imshow('image', image)
        cv2.imshow('orangeImage', orangeImage)
        cv2.waitKey(30)



def img_callback(msg):
    global img, width, height
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (640, 512))

    height, width, _ = img.shape
    

if __name__ == '__main__':
    rospy.init_node('findPath')
    bag = '/leftcam_bottom/image_raw/compressed'
    topic = '/bottom/left/image_raw/compressed'
    bot = '/bottom/left/image_raw/compressed'
    rospy.Subscriber(bag, CompressedImage, img_callback)
    find_bin()