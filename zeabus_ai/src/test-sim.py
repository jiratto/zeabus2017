#!/usr/bin/env python

import rospy
import math
import tf
import Queue as Queue
from geometry_msgs.msg import Twist, TwistStamped, Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool

def listToTwist (list):
	temp = Twist()
	temp.linear.x = list[0]
	temp.linear.y = list[1]
	temp.linear.z = list[2]
	temp.angular.x = list[3]
	temp.angular.y = list[4]
	temp.angular.z = list[5]
	return temp

def drive (list):
	pub (listToTwist(list))

def stop (time):
	pub (listToTwist([0, 0, 0, 0, 0, 0]))
	rospy.sleep (time)

def pub (tw):
	for i in xrange(5):
		command.publish (tw)
		rospy.sleep (0.05)

if __name__ == '__main__':
	global command
	command = rospy.Publisher ('/cmd_vel', Twist, queue_size = 10)
	drive ([1, 0, 0, 0, 0, 0])