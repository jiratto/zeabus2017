#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from AIControl import AIControl
import depth as const

class GateMission (object):

    def __init__ (self):
        print "Now do gate"
        self.aicontrol = AIControl()

    def run (self):
        print 'drive z'
        self.aicontrol.drive_z (const.PATH_DETECTING_DEPTH)
        self.aicontrol.drive_x (5)
        return

if __name__ == '__main__':
    print 'start gate'
    rospy.init_node('gate_ai', anonymous=True)
    self.run ()
    print "finish gate"
