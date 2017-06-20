#!/usr/bin/env python

import rospy
import math
from AIControl import AIControl

if __name__ == '__main__':
	print 'HELLO'
	rospy.sleep (2)
	aicontrol = AIControl ()

	rospy.init_node ('testMethod')
	print 'Init complete'

	# data = []
	# for i in xrange(10):
	# 	aicontrol.drive_yaxis (1)
	# 	rospy.sleep (0.2)
	# 	data.append (1)

	# for i in xrange(10):
	# 	aicontrol.drive_zaxis (0.5)
	# 	rospy.sleep (0.2)
	# 	data.append (0.5)

	# aicontrol.trackback (data, 0.2)
	# while True:
	# 	aicontrol.stop (1)

	# aicontrol.drive_xaxis (5)
	# aicontrol.drive_xaxis (-5)
	# rospy.sleep (10)

	# aicontrol.drive_yaxis (5)
	# aicontrol.drive_xaxis (-5)
	# rospy.sleep (0.5)

	# print aicontrol.get_position ()

	# aicontrol.stop (1)
	# aicontrol.fix_zaxis (-1.0)
	# rospy.sleep (5)
	# print 'drive z la na eiei'

	# aicontrol.go_to_xyz (10, 10, -5)
	# rospy.sleep (1)
	# print 'Go to XYZ complete'

	# aicontrol.drive ([-1, -1, 1, 0, 0, 0])
	# rospy.sleep (2)
	# aicontrol.stop (0.5)

	aicontrol.stop (2)
	aicontrol.turn_yaw_relative (45)
	rospy.sleep (2)
	aicontrol.turn_yaw_relative (-15)
	rospy.sleep (2)

	# aicontrol.turn_yaw_absolute (70)
	# rospy.sleep (5)
	# aicontrol.turn_yaw_absolute (-40)
	# rospy.sleep (5)

	# aicontrol.stop (5)