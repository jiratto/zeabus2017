#!/usr/bin/env python

import rospy
import math
from AIControl import AIControl

if __name__ == '__main__':
	rospy.sleep (2)
	aicontrol = AIControl ()

	aicontrol.drive_xaxis (5)
	aicontrol.drive_xaxis (-5)
	rospy.sleep (0.5)

	aicontrol.drive_yaxis (5)
	aicontrol.drive_xaxis (-5)
	rospy.sleep (0.5)

	aicontrol.drive_zaxis (-1)
	rospy.sleep (0.5)

	aicontrol.drive_xyaxis (1, 1, -1)
	rospy.sleep (0.5)

	aicontrol.go_to_xyz (3, 3, -3)
	rospy.sleep (0.5)

	aicontrol.drive ([1, 1, 1, 0, 0, 0])
	rospy.sleep (0.5)

	aicontrol.turn_yaw_relative (15)
	aicontrol.turn_yaw_relative (-15)
	rospy.sleep (0.5)

	aicontrol.turn_yaw_absolute (25)
	aicontrol.turn_yaw_absolute (-25)
	rospy.sleep (0.5)

	aicontrol.turn_yaw (30)
	rospy.sleep (0.5)

	aicontrol.stop (5)

	aicontrol.goto (10, 10, -2, 1)
	rospy.sleep (0.5)

	while True:
		aicontrol.stop (1)
		rospy.sleep (0.1)