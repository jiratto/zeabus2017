#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, String
from AIControl import AIControl
import Direction as dir

class Gate (object):

    def __init__ (self):
        print "Start Mission Gath"

        # rospy.init_node ('gate_node')

        self.aicontrol = AIControl ()
    
	def run (self):
		print 'TURN TO GATE DIRECTION'

		self.aicontrol.turn_yaw_absolute (dir.GATE_DIRECTION)
		rospy.sleep (2)

		print 'GO FORWARD'

		self.aicontrol.drive_xaxis (1)
		rospy.sleep (10)

if __name__ == '__main__':
	gate = Gate ()
	gate.run ()