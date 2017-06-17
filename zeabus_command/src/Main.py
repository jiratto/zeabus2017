#!/usr/bin/env python

import rospy
import math
from Pinger import Pinger
from AIControl import AIControl

if __name__ == '__main__':
	rospy.sleep (2)
	rospy.init_node ('command')
	print 'init node command complete'

	pinger = Pinger ()
	status = pinger.ping_check ()
	print 'Finish pinger: ', status

	aicontrol = AIControl ()
	aicontrol.stop (20)
	print 'Finis command'
