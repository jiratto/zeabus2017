#!/usr/bin/env python

import rospy
import math
from AIControl import AIControl
from pingping import PingerMission

if __name__ == '__main__':
	print 'HELLO'
	aicontrol = AIControl ()
	pingping = PingerMission ()	

	rospy.init_node ('testMethod')
	print 'Init complete'
	
	pingping.ping_check ()
	print 'FINISH'
