#!/usr/bin/env python
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
import rospy
import numpy as np
img = None


def callback(msg):
    global img
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.resize(cv2.imdecode(
        arr, 1), (640, 320))


def main():
    global img
    while not rospy.is_shutdown():
        if img is None:
            continue
        imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, thresh = cv2.threshold(imgray, 127, 255, 0)
        _, contours, _ = cv2.findContours(
            thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        imgray = img
        LIST = imgray.copy()
        cv2.drawContours(LIST, contours, -1, (255, 255, 255), 2)
        _, contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        EXT = imgray.copy()
        cv2.drawContours(EXT, contours, -1, (255, 255, 255), 2)
        _, contours, _ = cv2.findContours(
            thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        CCOMP = imgray.copy()
        cv2.drawContours(CCOMP, contours, -1, (255, 255, 255), 2)
        _, contours, _ = cv2.findContours(
            thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        TREE = imgray.copy()
        cv2.drawContours(TREE, contours, -1, (255, 255, 255), 2)
        cv2.imshow('LIST', LIST)
        cv2.imshow('EXT', EXT)
        cv2.imshow('CCOMP', CCOMP)
        cv2.imshow('TREE', TREE)
        k = cv2.waitKey(1) & 0xff
        if k == ord('q'):
            break
        rospy.sleep(0.1)
    cv2.destroyAllWindows()
if __name__ == '__main__':
    rospy.init_node('hierarchy')
    topic = "/top/center/image_rect_color/compressed"
    # topic = "/bottom/left/image_raw/compressed"
    rospy.Subscriber(topic, CompressedImage, callback)
    main()
