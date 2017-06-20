#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
	rospy.init_node('testForward')
	command = rospy.Publisher('/zeabus/cmd_vel', Twist, queue_size=10)
	t = Twist()
	t.linear.x = 0.5
	t.linear.y = 0
	t.linear.z = 0
	t.angular.x = 0
	t.angular.y = 0
	t.angular.z = 0
	while not rospy.is_shutdown():
		command.publish(t)
		
