#!/usr/bin/env python

import rospy
import math
from AIControl import AIControl

if __name__ == '__main__':
    aiControl = AIControl ()
    rospy.init_node ('stop_node')
    aiControl.stop (1)
    rospy.sleep (1)