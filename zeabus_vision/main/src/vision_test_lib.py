#!/usr/bin/env python
import cv2
import numpy as np
import math
import rospy
from vision_lib import *

if __name__ == '__main__':
    rospy.init_node('testlib')
    img = cv2.imread(
        '/home/zeabus/Pictures/Screenshot from 2017-06-21 12-35-00.png', 1)
    im = delete_color(img, 'yeljlow', 'top')

    _, contours, hierarchy = cv2.findContours(
        im, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    ct = 0
    for c in contours:
        M = cv2.moments(c)

        # print(M)
        if cut_contours(M, 1920, 1080, 50):
            print'================================='
            ct += 1
            cv2.drawContours(img, [c], 0, (255, 255, 0), 1)
        else:
            cv2.drawContours(img, [c], 0, (255, 5, 100), 1)
        print ct
    cv2.imshow('res', img)
    # cv2.imshow('res1', im)
    cv2.waitKey(300000)
