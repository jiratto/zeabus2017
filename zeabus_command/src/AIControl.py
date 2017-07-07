#!/usr/bin/env python

import rospy
import math
import tf
import Queue as Queue
from geometry_msgs.msg import Twist, TwistStamped, Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
from controller.srv import drive_x


class AIControl ():
	def __init__ (self):
		self.isFixPosition = True 			# use in fix_position call back function
		self.err = 0.1						# error constant
		self.pose = Pose ()					# pose varible type Pose
		# x, y, z in linear and angular of auv from /auv/
		self.auvState = [0, 0, 0, 0, 0, 0]
		# boolean to check turn state subscribe from /controller/is_at_fix_orientation
		self.stopTurn = True

		# subscribe from /auv/state call back to set_position method
		rospy.Subscriber ('/auv/state', Odometry, self.set_position)
		# subscribe to check Did you reach the position that you want ?
		rospy.Subscriber ('/zeabus_controller/is_at_fix_position',
						 Bool, self.fix_position)
		rospy.Subscriber ('/zeabus_controller/is_at_fix_orientation',
						 Bool, self.check_turn)		# subscribe to check turn state

		# publish twist x, y, z linear and angular to /cmd_vel
		self.command = rospy.Publisher ('/zeabus/cmd_vel', Twist, queue_size=10)
		self.zAxisNow = rospy.Publisher (
			'/fix/abs/depth', Float64, queue_size=10)		# publish z to fix depth
		self.fixPoint = rospy.Publisher (
			'/cmd_fix_position', Point, queue_size=10)		# publish x, y, z
		self.turnYawRelative = rospy.Publisher(
			'/fix/rel/yaw', Float64, queue_size=10)  # publish turn angle relative mode
		self.turnYawAbsolute = rospy.Publisher (
			'/fix/abs/yaw', Float64, queue_size=10)  # publish turn angle absolute mode

		# turn yaw relative by service
		# rospy.wait_for_service ('fix_rel_x_srv')
		# drive x [forward or backword] by service
		# self.driveXYService = rospy.ServiceProxy ('fix_rel_x_srv', drive_x)
		# self.wait_for_subscriber ()											# method check subscribtion

	###### Start check connection of subscriber ######
	def wait_for_subscriber (self, check_interval=0.3):
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
	###### End check connection ######

	###### Start set environment ######
	# subscribe from /auv/state
	def set_position (self, data):
		self.pose = data.pose.pose
		pose = data.pose.pose
		temp = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
		euler_angular = tf.transformations.euler_from_quaternion (temp)
		self.auvState[0] = pose.position.x
		self.auvState[1] = pose.position.y
		self.auvState[2] = pose.position.z
		self.auvState[3] = euler_angular[0]
		self.auvState[4] = euler_angular[1]
		self.auvState[5] = euler_angular[2]

	# call back check fix position
	def fix_position (self, data):
		self.isFixPosition = data.data

	# call back check finish of turn
	def check_turn (self, data):
		self.stopTurn = data.data

	# convert list to twist
	def list_to_twist (self, list):
		temp = Twist ()
		temp.linear.x = list[0]
		temp.linear.y = list[1]
		temp.linear.z = list[2]
		temp.angular.x = list[3]
		temp.angular.y = list[4]
		temp.angular.z = list[5]
		return temp

	# publish to cmd_vel
	def published (self, tw):
		print 'linear x:%f y:%f z:%f' % (tw.linear.x, tw.linear.y, tw.linear.z)
		print 'angular x:%f y:%f z:%f' % (tw.angular.x, tw.angular.y, tw.angular.z)
		for i in xrange(5):
			self.command.publish (tw)
			rospy.sleep (0.05)

	# get current position from /auv/state
	def get_position (self):
		return self.auvState
	###### End set environment ######

	###### Start move auv command ######
	# check Did we reach the point ?
	def wait_reach_fix_position (self, delay=0.1, check_interval=0.1, timeout_threshold=10):
		rospy.sleep (delay)
		waitedTime = 0
		while not rospy.is_shutdown () and not self.isFixPosition and waitedTime < timeout_threshold:
			waitedTime += check_interval
			rospy.sleep (check_interval)

	# forward (+) or backward (-)
	def drive_xaxis (self, x):
		# self.driveXYService (x, 0)
		# self.wait_reach_fix_position ()
		# print 'Drive x : %f' % x
		self.drive ([x, 0, 0, 0, 0, 0])

	# slide left (+) or right (-)
	def drive_yaxis (self, y):
		# self.driveXYService (0, y)
		# self.wait_reach_fix_position ()
		# print 'Drive y : %f' % y
		self.drive ([0, y, 0, 0, 0, 0])

	# up (+) or down (-)
	def drive_zaxis (self, z):
		self.drive ([0, 0, z, 0, 0, 0])

	def drive_x_rel (self, dis):
		li = self.get_position ()
		x = li[0]
		y = li[1]
		yaw = li[5]

		while x is None and y is None:
			rospy.sleep (0.01)

		start_x = x
		start_y = y
		x_dest = start_x + (dis * math.cos (yaw))
		y_dest = start_y + (dis * math.sin (yaw))
		print ('x start at: ', start_x)
		print ('y start at: ', start_y)
		print ('x dest at: ', x_dest)
		print ('y dest at: ', y_dest)

		if dis > 0:
			vx = 0.1
		elif dis < 0:
			vx = -0.1
		else:
			return

		self.drive_xaxis (vx)
		rospy.sleep (0.5)

		while not rospy.is_shutdown ():
			li = self.get_position ()
			x = li[0]
			y = li[1]

			while x is None and y is None:
				rospy.sleep (0.01)

			diff_x = abs (x - start_x)
			diff_y = abs (y - start_y)
			diff_total = math.sqrt (math.pow (diff_x, 2) + math.pow (diff_y, 2))

			if diff_total <= abs (dis):
				vx = (dis / diff_total) / 10
				self.drive_xaxis (vx)
				rospy.sleep (0.5)
				print ('present dist: ', diff_total)

			else:
				self.stop (2)
				print ('finish x: ', self.get_position ()[0])
				print ('finish y: ', self.get_position ()[1])
				break
			# self.stop (0.5)
		print ('Drive X_Rel complete')

	def drive_y_rel (self, dis):
		li = self.get_position ()
		x = li[0]
		y = li[1]
		yaw = li[5]

		while x is None and y is None:
			rospy.sleep (0.01)

		start_x = x
		start_y = y
		x_dest = start_x + (dis * math.cos (yaw + 90))
		y_dest = start_y + (dis * math.sin (yaw + 90))
		print ('x start at: ', start_x)
		print ('y start at: ', start_y)
		print ('x dest at: ', x_dest)
		print ('y dest at: ', y_dest)

		if dis > 0:
			vy = 0.1
		elif dis < 0:
			vy = -0.1
		else:
			return

		self.drive_yaxis (vy)
		rospy.sleep (0.5)

		while not rospy.is_shutdown ():
			li = self.get_position ()
			x = li[0]
			y = li[1]

			while x is None and y is None:
				rospy.sleep (0.01)

			diff_x = abs (x - start_x)
			diff_y = abs (y - start_y)
			diff_total = math.sqrt (math.pow (diff_x, 2) + math.pow (diff_y, 2))

			if diff_total <= abs (dis):
				vy = (dis / diff_total) / 10
				self.drive_yaxis (vy)
				rospy.sleep (0.5)
				print ('present dist: ', diff_total)

			else:
				self.stop (2)
				print ('finish x: ', self.get_position ()[0])
				print ('finish y: ', self.get_position ()[1])
				break
		print ('Drive complete')

	def fix_zaxis (self, z):
		z_dis = Float64 (z)
		for i in xrange (3):
			self.zAxisNow.publish (z_dis)
			rospy.sleep (0.2)
		self.wait_reach_fix_position (timeout_threshold=10)
		self.stop (1)
		# for i in xrange (3):
		# 	self.zAxisNow.publish (z_dis)
		# 	rospy.sleep (0.2)
		print 'drive z complete'

	def go_to_xyz (self, x, y, z):
		point = Point ()
		point.x = Float64 (x)
		point.y = Float64 (y)
		point.z = Float64 (z)
		self.fixPoint.publish (point)
		self.wait_reach_fix_position ()

	def drive (self, list):
		self.published (self.list_to_twist (list))

	def turn_yaw_relative (self, degree):
		rad = math.radians (degree)
		rad = Float64 (rad)
		self.turnYawRelative.publish (rad)
		while not self.stop_turn ():
			rospy.sleep (0.1)
		print ('turn yaw relative: ', rad)

	def turn_yaw_absolute (self, degree):
		rad = math.radians (degree)
		rad = Float64 (rad)
		self.turnYawAbsolute.publish (rad)
		while not self.stop_turn ():
			rospy.sleep (0.1)
		print ('turn yaw absolute: ', rad)

	def stop (self, time):
		stopList = [0, 0, 0, 0, 0, 0]
		self.published (self.list_to_twist (stopList))
		rospy.sleep (time)

	def stop_turn (self):
		return self.stopTurn

	def trackback (self, data, time):
		for i in xrange (len(data)):
			if i % 2 == 0:
				self.drive_yaxis (-data[i])
				rospy.sleep (time)
			else:
				if data[i] < 0:
					self.drive_zaxis (-data[i]/10)
					rospy.sleep (time)
				else:
					self.drive_zaxis (1)
					rospy.sleep (0.05)
		self.stop (5)

	# barrel roll movement
	def roll (self, time):
		q = Queue.Queue ()
		rotate_45 = [0.3826834, 0, 0, 0.9238795]
		cmd_vel_publisher = rospy.Publisher ('/zeabus/cmd_vel', Twist, queue_size=10)
		fix_orientation_publisher = rospy.Publisher (
			'/cmd_fix_orientation', Quaternion, queue_size=10)

		# calculate trajectory point and put to queue
		start_orientation = tf.transformations.quaternion_from_euler (
			0, 0, self.auvState[5])
		x = start_orientation

		l = []
		for i in range (0, 8):
			x = tf.transformations.quaternion_multiply (x, rotate_45)
			l.append (x)
		for i in range (0, time):
			for quat in l:
				q.put (quat)

		q.put (start_orientation)

		twist = Twist ()
		twist.linear.x = 1.5
		cmd = 0
		last_cmd = q.qsize ()
		print ("START ROLLING")
		r = rospy.Rate (2)
		while not q.empty () and not rospy.is_shutdown ():
			quat = q.get ()
			cmd_vel_publisher.publish (twist)
			fix_orientation_publisher.publish (*quat)
			cmd = cmd + 1
			print cmd, "/", last_cmd
			r.sleep()
		print ("END OF ROLLING")
		twist.linear.x = 0
		cmd_vel_publisher.publish (twist)
	###### End move auv command ######

	###### Start navigation function ######
	def distance (self, x, y):
		return sum (map (lambda x, y: (x - y) ** 2, x, y)) ** 0.5

	def twopi (self, rad):
		if rad <= 0:
			return abs (rad)
		else:
			return 2 * math.pi - rad

	def w_yaw (self, setyaw):
		degi = self.twopi (self.auvState[5])
		degf = self.twopi (setyaw)
		diff = (degi - 2 * math.pi - degf, degi -
				degf, 2 * math.pi - degf + degi)
		diff = min (diff, key=abs)
		diff *= 2
		if diff >= 0:
			return abs (diff)
		return -abs (diff)

	def delta_radians (self, x, y, bit):
		radians = math.atan2 (
			(x - self.auvState[0]) * bit, (y - self.auvState[1]) * bit)
		radians -= math.pi / 2
		radians *= -1
		if radians > math.pi:
			return radians - 2 * math.pi
		if radians < -math.pi:
			return radians + 2 * math.pi
		return radians
	###### End navigation function ######

	###### Start image function ######
	# check Are we at center now ?
	def is_center (self, point, xMin, xMax, yMin, yMax):
		if (xMin <= point[0] <= xMax) and (yMin <= point[1] <= yMax):
			return True
		return False

	# adjust value in scope that we set min-max of negative and positive value
	def adjust (self, value, nMin, nMax, pMin, pMax):
		if value > 0:
			if value > pMax:
				return pMax
			if value < pMin:
				return pMin
		elif value < 0:
			if value > nMax:
				return nMax
			if value < nMin:
				return nMin
		return value
	###### End image function ######

	###### Start other function ######
	# check fail
	def is_fail (self, count):
		if count > 0:
			return False
		return True
	###### End other function ######

	def average (self, data):
		summary = 0
		for i in data:
			summary += i
		return summary / len(data)

if __name__ == '__main__':
	aicontrol = AIControl ()
