#!/usr/bin/env python

import rospy
import math
import tf
import Queue as Queue
from geometry_msgs.msg import Twist, TwistStamped, Pose, PoseStmped, Point, Quternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
from controller.srv import drive_x

class AIControl():
	def __init__ (self):
		self.isFixPostion = True
		self.err = 0.1
		self.pose = Pose()
		self.auvState = [0, 0, 0, 0, 0, 0]
		self.stopTurn = True

		rospy.Subscriber ('/auv/state', Odometry, self.set_position)
		rospy.Subscriber ('/controller/is_at_fix_position', Bool, self.fix_position)
		rospy.Subscriber ('/controller/is_at_fix_orientation', Bool, self.check_turn)

		self.command = rospy.Publisher ('/cmd_vel', Twist, queue_size = 10)
		self.zAxisNow = rospy.Publisher ('/fix/abs/depth', Float64, queue_size = 10)
		self.fixPoint = rospy.Publisher ('/cmd_fix_position', Point, queue_size = 10)
		self.turnYawRelative = rospy.Publisher ('/fix/rel/yaw', Float64, queue_size = 10)
		self.turnYawAbsolute = rospy.Publisher ('/fix/abs/yaw', Float64, queue_size = 10)

		rospy.wait_for_service ('fix_rel_x_srv')
		self.driveXService = rospy.ServiceProxy ('fix_rel_x_srv', drive_x)
		self.wait_for_subscriber ()

	def set_position (self, data):
		pose = data.pose.pose

	def fix_position (self, data):
		self.isFixPostion = data.data

	def check_turn (self, data):
		self.stopTurn = data.data

	def wait_for_subscriber (self, check_interval = 0.3):
		finish = False
		while not rospy.is_shutdown () and not finish:
			count = 0
			print count

			if self.command.get_num_connections () > 0:
				count += 1
			if self.zAxisNow.get_num_connections () > 0:
				count += 1
			if self.fixPoint.get_num_connections () > 0:
				count += 1
			if self.turnYawRelative.get_num_connections () > 0:
				count += 1
			if self.turnYawAbsolute.get_num_connections () > 0:
				count += 1

			if count > 4:
				finish = True

			if not finish:
				rospy.sleep (check_interval)
				print 'Some subscribtion not connected ;____;'

		print 'All subscribe complete !'