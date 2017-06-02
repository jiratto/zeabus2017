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
		self.isFixPostion = True 			# use in fix_position call back function
		self.err = 0.1						# error constant
		self.pose = Pose()					# pose varible type Pose
		self.auvState = [0, 0, 0, 0, 0, 0]	# x, y, z in linear and angular of auv from /auv/ 
		self.stopTurn = True 				# boolean to check turn state subscribe from /controller/is_at_fix_orientation

		rospy.Subscriber ('/auv/state', Odometry, self.set_position)						# subscribe from /auv/state call back to set_position method
		rospy.Subscriber ('/controller/is_at_fix_position', Bool, self.fix_position)		# subscribe to check Did you reach the position that you want ?
		rospy.Subscriber ('/controller/is_at_fix_orientation', Bool, self.check_turn)		# subscribe to check turn state 

		self.command = rospy.Publisher ('/cmd_vel', Twist, queue_size = 10)					# publish twist x, y, z linear and angular to /cmd_vel
		self.zAxisNow = rospy.Publisher ('/fix/abs/depth', Float64, queue_size = 10)		# publish z to fix depth
		self.fixPoint = rospy.Publisher ('/cmd_fix_position', Point, queue_size = 10)		# publish x, y, z
		self.turnYawRelative = rospy.Publisher ('/fix/rel/yaw', Float64, queue_size = 10)	# publish turn angle relative mode
		self.turnYawAbsolute = rospy.Publisher ('/fix/abs/yaw', Float64, queue_size = 10)	# publish turn angle absolute mode

		rospy.wait_for_service ('fix_rel_x_srv')							# turn yaw relative by service
		self.driveXService = rospy.ServiceProxy ('fix_rel_x_srv', drive_x)	# drive x [forward or backword] by service
		self.wait_for_subscriber ()											# method check subscribtion

	# subscribe from /auv/state
	def set_position (self, data):
		self.pose = data.pose.pose
		pose = data.pose.pose
		temp = (pose.orientation.y, pose.orientation.z, pose.orientation.w)
		euler_angular = tf.transformations.euler_from_quaternion (temp)
		self.auvState[0] = pose.position.x
		self.auvState[1] = pose.position.y
		self.auvState[2] = pose.position.z
		self.auvState[3] = euler_angular[0]
		self.auvState[4] = euler_angular[1]
		self.auvState[5] = euler_angular[2]

	# call back check fix position
	def fix_position (self, data):
		self.isFixPostion = data.data

	# call back check finish of turn
	def check_turn (self, data):
		self.stopTurn = data.data

	# check connection
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

	# convert list to twist
	def list_to_twist (self, list):
		temp = Twist()
		temp.linear.x = list[0]
		temp.linear.y = list[1]
		temp.linear.z = list[2]
		temp.angular.x = list[3]
		temp.angular.y = list[4]
		temp.angular.z = list[5]
		return temp

	# publish to cmd_vel
	def published (self, tw):
		print 'linear x:%f y:%f z:%f'%(tw.linear.x, tw.linear.y, tw.linear.z)
		print 'angular x:%f y:%f z:%f'%(tw.angular.x, tw.angular.y, tw.angular.z)

		self.command.publish (tw)
		rospy.sleep (0.05)

	def get_position (self): 	
		return self.auvState
