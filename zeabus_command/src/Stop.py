#!/usr/bin/env python

import rospy
import math
from AIControl import AIControl

if __name__ == '__main__':
    aicontrol = AIControl ()
    rospy.init_node ('stop_node')
    aicontrol.stop (1)
    rospy.sleep (1)
    # aicontrol.fix_zaxis (-1)