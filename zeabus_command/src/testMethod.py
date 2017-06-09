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

	# aicontrol.drive_xaxis (5)
	# aicontrol.drive_xaxis (-5)
	# rospy.sleep (0.5)

	# aicontrol.drive_yaxis (5)
	# aicontrol.drive_xaxis (-5)
	# rospy.sleep (0.5)

	# aicontrol.drive_zaxis (3)
	# rospy.sleep (0.5)
	# print 'drive z la na eiei'

	# aicontrol.go_to_xyz (10, 10, -5)
	# rospy.sleep (1)
	# print 'Go to XYZ complete'

	# aicontrol.drive ([-1, -1, 1, 0, 0, 0])
	# rospy.sleep (2)
	# aicontrol.stop (0.5)

	# aicontrol.turn_yaw_relative (15)
	# rospy.sleep (0.5)
	# aicontrol.turn_yaw_relative (-15)
	# rospy.sleep (0.5)

	# aicontrol.turn_yaw_absolute (70)
	# rospy.sleep (5)
	# aicontrol.turn_yaw_absolute (-40)
	# rospy.sleep (5)

	# aicontrol.stop (5)