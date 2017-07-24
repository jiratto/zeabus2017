#!/usr/bin/env python

import rospy
import math
import Direction as dir
from Pinger import Pinger
from Path import Path
from Bouy2 import Bouy
from Navigate2 import Navigate
from AIControl import AIControl
from modbus_ascii_ros.msg import Switch

def start (switch):
	global check
	global ever_turn_on
	if switch.motor_switch == False:
		check = False
		if ever_turn_on:
			rospy.signal_shutdown ('Turn off switch')
	else:
		check = True
		ever_turn_on = True

if __name__ == '__main__':
	rospy.sleep (2)
	global check
	global ever_turn_on
	ever_turn_on = False
	rospy.Subscriber ("/switch/data", Switch, start, queue_size = 1)
	rospy.init_node ('main_node')
	print 'init node main_node complete'

	aicontrol = AIControl ()

	## Init task ##
	gate = Gate ()
	path = Path ()
	bouy = Bouy ()
	navigate = Navigate ()
	pinger = Pinger ()
	binn = Binn ()
	squid = Squid ()
	tower = Tower ()

	## TASK: GATE ##
	gate.run ()

	## TASK: PATH ##
	if path.find ():
		print 'FIND PATH COMPLETE'
	else:
		print 'FIND PATH FAILED'

		print 'TURN TO BOUY'
		aicontrol.turn_yaw_absolute (dir.BOUY_DIRECTION)
		rospy.sleep (2)

	## TASK: BOUY ##
	if bouy.go_bouy ():
		print 'BOUY COMPLETE'
	else:
		print 'BOUY FAILED'

		print 'TURN TO PATH_2'
		aicontrol.turn_yaw_absolute (dir.PATH2_DIRECTION)
		rospy.sleep (2)

	## TASK: PATH ##
	if path.find ():
		print 'FIND PATH_2 COMPLETE'
	else:
		print 'FIND PATH_2 FAILED'

		print 'TURN TO NAVIGATE'
		aicontrol.turn_yaw_absolute (dir.NAVIGATE_DIRECTION)
		rospy.sleep (2)

	## TASK: NAVIGATE ##
	if navigate.do_navigate ():
		print 'NAVIGATE COMPLETE'
	else:
		print 'NAVIGATE FAIL'

		aicontrol.turn_yaw_absolute (dir.NAVIGATE_DIRECTION)
		rospy.sleep (2)
		
		## DRIVE FORWARD ##

	status = pinger.ping_check ()
	# print 'Finish pinger: ', status

	rospy.sleep (5)

	if status == 'bin':
		binn.run ()
		squid.run ()
	elif status == 'tower':
		tower.run ()
		