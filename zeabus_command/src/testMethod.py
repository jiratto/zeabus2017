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

	# print 'Forward'
	# aicontrol.drive_xaxis (1)
	# rospy.sleep (10)

	# print 'Backward'
	# aicontrol.drive_xaxis (-1)
	# rospy.sleep (40)

	# print 'Slide left'
	# aicontrol.drive_yaxis (1)
	# rospy.sleep (5)

	# print 'Slide right'
	# aicontrol.drive_yaxis (-1)
	# rospy.sleep (5)	

	# print 'Fix Z to 3'
	# aicontrol.fix_zaxis (-2.5)

	# print 'Fix Z to 0'
	# aicontrol.fix_zaxis (-0.2)

	# print 'Fix Z to -0.5'
	# aicontrol.fix_zaxis (-0.5)	
	aicontrol.stop (1)
	print 'Turn yaw relative 45'
	aicontrol.turn_yaw_relative (44)
	rospy.sleep (3)

	# print 'Go to XYZ'
	# aicontrol.go_to_xyz (4, 2.06, 1.54)
	# rospy.sleep (10)
	
	# aicontrol.stop (2)	
	# print 'Turn yaw relative -27'
	# aicontrol.turn_yaw_relative (-27)
	# rospy.sleep (3)

	# aicontrol.stop (2)	
	# print 'Turn yaw relative 90'
	# aicontrol.turn_yaw_relative (-90)
	# rospy.sleep (3)

	# print 'Turn yaw absolute 0'
	# aicontrol.turn_yaw_absolute (0)
	# rospy.sleep (1)	

	# print 'FINISH STOP 5 sec'
	# aicontrol.stop (5)
	# rospy.sleep (1)

	# rospy.sleep (5)
	# print 'Turn yaw relative 60'
	# aicontrol.turn_yaw_relative (60)

	# rospy.sleep (5)
	# print 'Turn yaw relative -60'
	# aicontrol.turn_yaw_relative (-60)

	# rospy.sleep (2)
	# aicontrol.drive_yaxis (2)
	# rospy.sleep (0.5)
	# aicontrol.drive_zaxis (-1)
	# rospy.sleep (0.5)

	# rospy.sleep (2)
	# data = [2, -1]
	# aicontrol.trackback (data, 1)

	# start_x = aicontrol.get_position ()[0]
	# aicontrol.drive_x_rel (2)
	# dest_x = aicontrol.get_position ()[0]
	# aicontrol.drive_x_rel (-2)

	aicontrol.stop (1)
