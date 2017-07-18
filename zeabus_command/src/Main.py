#!/usr/bin/env python

import rospy
import math
import Direction as dir
from Pinger import Pinger
from Path import Path
from Bouy2 import Bouy
from Navigate2 import Navigate
from AIControl import AIControl

if __name__ == '__main__':
	aicontrol = AIControl ()

	rospy.init_node ('main_node')
	print 'init node main_node complete'

	## TASK: GATE ##
	gate = Gate ()
	gate.run ()

	## TASK: PATH ##
	path = Path ()
	if path.find ():
		print 'FIND PATH COMPLETE'
	else:
		print 'FIND PATH FAILED'

		print 'TURN TO BOUY'
		aicontrol.turn_yaw_absolute (dir.BOUY_DIRECTION)
		rospy.sleep (2)

	## TASK: BOUY ##
	bouy = Bouy ()

	if bouy.go_bouy ():
		print 'BOUY COMPLETE'
	else:
		print 'BOUY FAILED'

		print 'TURN TO PATH_2'
		aicontrol.turn_yaw_absolute (dir.PATH2_DIRECTION)
		rospy.sleep (2)

	## TASK: PATH ##
	path = Path ()
	if path.find ():
		print 'FIND PATH_2 COMPLETE'
	else:
		print 'FIND PATH_2 FAILED'

		print 'TURN TO NAVIGATE'
		aicontrol.turn_yaw_absolute (dir.NAVIGATE_DIRECTION)
		rospy.sleep (2)

	## TASK: NAVIGATE ##
	navigate = Navigate ()
	if navigate.do_navigate ():
		print 'NAVIGATE COMPLETE'
	else:
		print 'NAVIGATE FAIL'

		aicontrol.turn_yaw_absolute (dir.NAVIGATE_DIRECTION)
		rospy.sleep (2)
		
		## DRIVE FORWARD ##


	pinger = Pinger ()
	status = pinger.ping_check ()
	# print 'Finish pinger: ', status

	rospy.sleep (5)

	if status == 'bin':
		binn = Binn ()
		binn.run ()
		squid = Squid ()
		squid.run ()
	elif status == 'tower':
		tower = Tower ()
		tower.run ()
		