#!/usr/bin/env python

import rospy
import math
from AIControl import AIControl
from geometry_msgs.msg import Twist
if __name__ == '__main__':
	print 'HELLO'
	rospy.sleep (2)
	aicontrol = AIControl ()

	rospy.init_node ('testMethod')
	pub = rospy.Publisher ('/zeabus/cmd_vel', Twist, queue_size=1)
	print 'Init complete'
	tw = Twist()
	tw.linear.x = 0.5
	tw.linear.y = 0
	tw.linear.z = 0
	tw.angular.x = 0
	tw.angular.y = 0
	tw.angular.z = 0


	for i in xrange (100):
		print i
		pub.publish (tw)
		rospy.sleep (0.1)
