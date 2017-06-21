#!/usr/bin/env python

import rospy
import math
from AIControl import AIControl
from zeabus_hydrophone.srv import hydro_info
from zeabus_hydrophone.msg import hydro_msg
from std_msgs.msg import String, Bool

class Pinger (object):
	def __init__ (self):
		print 'Pinger init'
		self.aicontrol = AIControl ()
		self.hydroData = hydro_msg ()
		self.check = Bool
		self.gotData = False

		rospy.Subscriber ('/hydro', hydro_msg, self.listening)

	def reset (self):
		resetHydrophone = rospy.Publisher ('/hydro_status', Bool, queue_size = 1)
		while resetHydrophone.get_num_connections () == 0:
			rospy.sleep (1)
		resetHydrophone.publish (True)

	def listening (self, data):
		self.gotData =  True
		self.hydroData = data

	def convert (self, azi):
		azi = -azi
		turn = azi - 45
		if turn < -180:
			turn = turn + 360
		return turn

	def check_data (self):
		return self.gotData

	def ping_check (self):
		print 'listening pinger'

		self.aicontrol.stop (1)
		self.aicontrol.fix_zaxis (-1)

		self.aicontrol.stop (5)
		self.reset ()
		self.aicontrol.stop (5)

		realDegree = self.convert (self.hydroData.azi)
		print 'azi: ', self.hydroData.azi

		driveXdistance = 7
		self.aicontrol.turn_yaw_relative (realDegree)

		goal = False
		count = 50

		while goal != True and not rospy.is_shutdown ():
			rospy.sleep (2)
			print 'listening pinger'

			print self.hydroData

			self.aicontrol.stop (1)
			if self.hydroData.distance != -999:
				realDegree = self.convert (self.hydroData.azi)
				print 'Real degree ngai la tor ', realDegree 
				if -10 < realDegree and realDegree < 10:
					self.aicontrol.drive_xaxis (driveXdistance)
					print 'drive'
					driveXdistance -= 0.5
					if driveXdistance <= 0:
						driveXdistance = 0.5
					rospy.sleep (5)
				else:
					turnnaja = self.aicontrol.adjust (realDegree, -10, -5, 5, 10)
					print turnnaja
					self.aicontrol.turn_yaw_relative (turnnaja)
					print 'turn'
					rospy.sleep (5)

			if self.hydroData.elv < 40:
				driveXdistance = 0.5

			if self.hydroData.stop:
				goal = True
				self.aicontrol.fix_zaxis (0)
				print "FUCKING FINISH !!"

		return goal

		# while goal != True and not rospy.is_shutdown () and not self.aicontrol.is_fail (count):
		# 	print 'listening pinger round: ', count

		# 	self.aicontrol.stop (1)

		# 	status = self.aicontrol.get_position ()
		# 	my_yaw = status[5]

		# 	if self.hydroData.distance != -999:
		# 		if self.aicontrol.stop_turn ():
		# 			realDegree = self.convert (self.hydroData.azi)
		# 			print 'yaw pinger: ', realDegree

		# 			if -10 < realDegree and realDegree < 10:
		# 				self.aicontrol.drive_xaxis (driveXdistance)
		# 				print 'drive'
		# 				driveXdistance -= 0.5
		# 				if driveXdistance <= 0:
		# 					driveXdistance = 0.5
		# 				rospy.sleep (5)
		# 			else:
		# 				self.aicontrol.turn_yaw_relative (realDegree)
		# 				rospy.sleep (10)
		# 				print 'turn'
		# 		else:
		# 			print 'still turn yaw'

		# 		print self.hydroData
		# 		rospy.sleep (10)

		# 		if self.hydroData.elv < 40:
		# 			driveXdistance = 0.5
		# 		if self.hydroData.stop:
		# 			print self.hydroData.elv
		# 			goal = True
		# 			self.aicontrol.drive_zaxis (0)
		# 			print "FUCKING FINISH !!"

		# 	count -= 1
		# return goal

if __name__ == '__main__':
	pinger = Pinger ()
	pinger.ping_check ()
