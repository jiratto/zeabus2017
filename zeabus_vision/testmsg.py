#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
import math
from sensor_msgs.msg import CompressedImage
from vision_lib import *
from zeabus_vision_srv_msg.msg import vision_msg_default
from zeabus_vision_srv_msg.srv import vision_srv_default

img = None
# width = 1280
# height = 1024

def find_path():
    global img, width, height
    res = vision_msg_default()
    print('1')
    
    while img is None:
        print("img: None")
        rospy.sleep(0.01)
        # continue
    print('2')
    res.appear = False
    res.area = 0
    res.x = 0
    res.y = 0
    res.angle = 0

    cv2.imshow('img', img)
    print('3')
    cv2.waitKey(1)
    print('4')
    return res
    
    
def img_callback(msg):
    global img
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(arr, 1), (640, 512))
 


def mission_callback(msg):
    print('mission_callback')
    return find_path()

if __name__ == '__main__':
    rospy.init_node('findPath')
    topic = '/leftcam_bottom/image_raw/compressed'
    rospy.Subscriber(topic, CompressedImage, img_callback)
    rospy.Service('vision', vision_srv_default(), mission_callback)
    rospy.spin()