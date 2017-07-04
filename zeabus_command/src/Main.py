#!/usr/bin/env python

import rospy
import math
from Pinger import Pinger
from Path import Path
from Navigate2 import Navigate
from AIControl import AIControl

if __name__ == '__main__':
	rospy.sleep (2)
	rospy.init_node ('main_node')
	print 'init node main_node complete'

	path = Path ()
	path.run ()
	rospy.sleep (2)
	navigate = Navigate ()
	navigate.run ()
	# pinger = Pinger ()
	# status = pinger.ping_check ()
	# print 'Finish pinger: ', status

	# rospy.sleep (5)

	# path = Path ()
	# path.run ()

	# aicontrol = AIControl ()
	# aicontrol.stop (2)
	# aicontrol.drive_xaxis (4)
	
	# print 'Start barrel roll'
	# aicontrol.roll (2)
	# aicontrol.stop (5)
	# print 'Finish command'
