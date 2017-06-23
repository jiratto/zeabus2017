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

	print 'Forward'
	aicontrol.drive_xaxis (1)
	rospy.sleep (3)

	print 'Backward'
	aicontrol.drive_xaxis (-1)
	rospy.sleep (3)

	print 'Slide left'
	aicontrol.drive_yaxis (1)
	rospy.sleep (3)

	print 'Slide right'
	aicontrol.drive_yaxis (-1)
	rospy.sleep (3)	

	print 'Fix Z to 3'
	aicontrol.fix_zaxis (-3)

	print 'Fix Z to 0'
	aicontrol.fix_zaxis (-0.2)

	print 'Fix Z to 1'
	aicontrol.fix_zaxis (-1)	

	print 'Turn yaw relative 45'
	aicontrol.turn_yaw_relative (45)
	rospy.sleep (1)

	print 'Turn yaw relative -15'
	aicontrol.turn_yaw_relative (-15)
	rospy.sleep (1)

	print 'Turn yaw absolute 0'
	aicontrol.turn_yaw_absolute (0)
	rospy.sleep (1)	

	print 'FINISH STOP 5 sec'
	aicontrol.stop (5)
	rospy.sleep (1)

