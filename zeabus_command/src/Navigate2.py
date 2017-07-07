#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, String
from zeabus_vision_srv_msg.msg import vision_msg_navigate
from zeabus_vision_srv_msg.srv import vision_srv_navigate
from AIControl import AIControl
import Depth as depth

class Navigate (object):

	def __init__ (self):
		print "Start Mission Navigate"

		rospy.init_node ('navigate_node')

		self.aicontrol = AIControl ()
		self.checkStateOne = 0
		self.checkStateThree = 0
		self.countStateZero = 20
		self.countStateOne = 30
		self.countStateTwo = 20
		self.countStateThree = 20
		self.oneLeg = 0 # check one leg loop case
		self.task = 'navigate'
		self.state = 0 # state go to navigate
		self.data = None
		self.angleBot = None
		self.pos = None

		navigate_srv = 'vision_navigate'
		rospy.wait_for_service (navigate_srv)
		self.detectNav = rospy.ServiceProxy (navigate_srv, vision_srv_navigate) 

	## state 0
	def check_found (self):
		zero = 0
		one = 0
		two = 0

		if self.aicontrol.is_fail (self.countStateZero):
			print 'STATE ZERO FAIL'

			self.state = -1

		for i in xrange (10):
			self.data = self.detectNav (String (self.task), String ('top'))
			self.data = self.data.data
			if self.data.numVertical >= 2:
				two += 1
			elif self.data.numVertical == 1:
				one += 1
			elif self.data.numVertical == 0:
				zero += 1
			rospy.sleep (0.1)

		if zero > one >= two:
			print 'NOT FOUND NAVIGATE'

			self.aicontrol.drive_xaxis (1)
			rospy.sleep (0.5)
			self.countStateZero -= 1
		else:
			print 'FOUND'

			self.state = 1

	## state 1
	def go_to_navigate (self):
		zero = 0
		one = 0
		two = 0
		mul = 0

		cx = [0] * 3
		cy = [0] * 3
		area = [0] * 3

		for i in xrange (10):
			self.data = self.detectNav (String (self.task), String ('top'))
			self.data = self.data.data
			if self.data.numVertical >= 2:
				two += 1
				cx[2] += self.data.cx
				cy[2] += self.data.cy
				area[2] += self.data.ratioArea
			elif self.data.numVertical == 1:
				one += 1
				area[1] += self.data.ratioArea
				mul += self.data.direction
			elif self.data.numVertical == 0:
				zero += 1
			rospy.sleep (0.1)
		
		if two > one >= zero:
			print 'TWO LEGS'

			self.oneLeg = 0
			avrCx = cx[2] / two

			if self.aicontrol.is_center ([avrCx, 0], -0.1, 0.1, -0.1, 0.1):
				print 'Center !!'
				# vx = (1 / area[2]) * 3
				self.aicontrol.drive_xaxis (1)
				rospy.sleep (8)
			else:
				print 'Drive to center'
				vy = self.aicontrol.adjust (avrCx, -0.6, -0.3, 0.3, 0.6)
				self.aicontrol.drive ([0, -vy, 0, 0, 0, 0])
				rospy.sleep (3)
		elif one > two >= zero:
			print 'ONE LEG'

			if self.aicontrol.is_fail (self.countStateOne):
				print 'STATE 1 FAIL'
				self.state = -1
				
			if mul > 0:
				vy = 0.4
			elif mul < 0:
				vy = -0.4
			else:
				return

			if self.oneLeg >= 8:
				self.aicontrol.drive_xaxis (1)
				rospy.sleep (3)
				self.oneLeg = 0
				return
			elif self.oneLeg >= 6:
				vy = self.aicontrol.adjust (vy, -0.2, -0.1, 0.1, 0.2)
			elif self.oneLeg >= 4:
				vy = self.aicontrol.adjust (vy, -0.3, -0.2, 0.2, 0.3)

			self.aicontrol.drive_yaxis (vy)
			rospy.sleep (2)
			self.countStateOne -= 1
			self.oneLeg += 1

		elif zero > one >= two:
			print 'ZERO'

			self.oneLeg = 0

			self.aicontrol.drive_xaxis (1)
			rospy.sleep (1)
			self.checkStateOne += 1
			if self.checkStateOne >= 3:
				self.aicontrol.fix_zaxis (depth.NAVIGATE_BOTTOM)
				rospy.sleep (1)
				self.aicontrol.stop (1)
				
				print 'State 2 : check bottom'
				self.state = 2
	
	## state 2
	def check_bottom (self):
		found = 0
		notFound = 0
		angle = []

		for i in xrange (10):
			self.data = self.detectNav (String (self.task), String ('bot'))
			self.data = self.data.data
			
			if -180 < self.data.angle < 180:
				angle.append (self.data.angle)
			
			if self.data.appear:
				found += 1
			else:
				notFound += 1

		if found > notFound:
			print 'FOUND'

			countTurn = 10

			while not self.aicontrol.is_fail (countTurn):
				angle = []
				for i in xrange (10):
					self.data = self.detectNav (String (self.task), String ('bot'))
					self.data = self.data.data
					
					if -180 <= self.data.angle <= 180:
						angle.append (self.data.angle)
				if len (angle) != 0:
					self.angleBot = self.aicontrol.average (angle)
				else:
					self.aicontrol.drive_xaxis (1)
					rospy.sleep (0.5)
					continue

				# self.aicontrol.drive_xaxis (-0.8)
				# rospy.sleep (3)
				# self.pos = self.aicontrol.get_position ()[0]
				# self.aicontrol.stop (0.5)
				# self.aicontrol.drive_xaxis (-0.8)
				# rospy.sleep (3)
				if -2 <= self.angleBot <= 2:
					break

				print ('Turn angle: ', self.angleBot)
				self.aicontrol.turn_yaw_relative (self.angleBot)
				rospy.sleep (2)
				self.aicontrol.stop (2)
				countTurn -= 1

			self.pos = self.aicontrol.get_position ()[0]
			self.aicontrol.stop (0.5)
			self.aicontrol.drive_xaxis (-1)
			rospy.sleep (5)
			self.aicontrol.fix_zaxis (depth.NAVIGATE_DETECTING)

			print ('State 3 : back and roll')
			self.state = 3
		else:
			print 'NOT FOUND'
			self.aicontrol.drive_xaxis (1)
			rospy.sleep (4)

			self.countStateTwo -= 1

			if self.aicontrol.is_fail (self.countStateTwo):
				print ('STATE 2_FAIL')
				self.state = -1

	## state 3
	def back_and_roll (self):
		zero = 0
		one = 0
		two = 0
		mul = 0
		cx = [0] * 3
		cy = [0] * 3
		area = [0] * 3

		if self.aicontrol.is_fail (self.countStateThree):
			print 'STATE 3 FAIL'
			self.state = -1

		for i in xrange (10):
			self.data = self.detectNav (String (self.task), String ('top'))
			self.data = self.data.data
			if self.data.numVertical >= 2:
				two += 1
				cx[2] += self.data.cx
				cy[2] += self.data.cy
				area[2] += self.data.ratioArea
			elif self.data.numVertical == 1:
				one += 1
				area[1] += self.data.ratioArea
				mul += self.data.direction
			elif self.data.numVertical == 0:
				zero += 1
			rospy.sleep (0.1)
		
		if two > one >= zero:
			print 'TWO LEGS'

			avrCx = cx[2] / two

			if self.aicontrol.is_center ([avrCx, 0], -0.1, 0.1, -0.1, 0.1):
				print 'Center !!'
				vx = (1 / area[2]) * 3
				self.aicontrol.drive_xaxis (vx)
				rospy.sleep (5)
			else:
				print 'Drive to center'
				vy = self.aicontrol.adjust (avrCx, -0.6, -0.4, 0.4, 0.6)
				self.aicontrol.drive_yaxis (-vy)
				rospy.sleep (3)

			# tmp = self.aicontrol.get_position ()
			# dis = abs (self.pos[0] - tmp[0])
			# print 'GO TO NAVIGATE BEFORE ROLL'
			# self.aicontrol.drive_x_rel (dis)
			# self.aicontrol.stop (1)

			# print 'ROLL'
			# self.aicontrol.drive_xaxis (1)
			# rospy.sleep (3)

			print 'NAVIGATE COMPLETE'
			self.state = 4
		elif one > two >= zero:
			print 'ONE LEG'

			if mul > 0:
				vy = 1
			elif mul < 0:
				vy = -1
			else:
				return
			self.aicontrol.drive_yaxis (vy)
			rospy.sleep (4)
			self.countStateThree -= 1

			# self.aicontrol.drive_xaxis (-0.8)
			# rospy.sleep (2)
			# self.countStateThree -= 1

		elif zero > one >= two:
			print 'ZERO'

			if self.checkStateThree < 3:
				self.aicontrol.drive_xaxis (1)
				rospy.sleep (2)
				self.checkStateThree += 1
			else:
				print 'FORWARD TO ROLL'

				self.aicontrol.fix_zaxis (depth.NAVIGATE_ROLL)
				# self.aicontrol.drive_xaxis (1)
				# rospy.sleep (3)
				x = self.aicontrol.get_position ()[0]
				print ('x: ', x)
				print ('dest_x: ', self.pos)
				self.aicontrol.drive_x_rel (self.pos - x)
				if x >= self.pos:
					print 'ROLL'
					# self.aicontrol.roll (2)
					# self.aicontrol.stop (1)
					self.aicontrol.drive_xaxis (0.7)
					rospy.sleep (10)
					print 'State 4 : Navigate complete'
					self.state = 4
			# self.aicontrol.drive_xaxis (-0.8)
			# rospy.sleep (2)
			# self.countStateThree -= 1

	def run (self):
		task = 'navigate'
		cam = 'top'

		self.aicontrol.fix_zaxis (depth.NAVIGATE_DETECTING)
		# self.aicontrol.drive_x_rel (5)
		self.aicontrol.drive_xaxis (1)
		rospy.sleep (10)

		while not rospy.is_shutdown ():
			if self.state == 0:
				self.check_found ()
			if self.state == 1:
				self.go_to_navigate ()
			elif self.state == 2:
				self.check_bottom ()
			elif self.state == 3:
				self.back_and_roll ()
			elif self.state == 4 or self.state == -1:
				# self.aicontrol.fix_zaxis (depth.FLOATING)
				break
			self.aicontrol.stop (0.5)
		self.aicontrol.stop (1)

	def bot (self):
		self.aicontrol.fix_zaxis (-1.7)
		self.check_bottom ()

if __name__ == '__main__':
	navigate = Navigate ()
	navigate.run ()
	# navigate.bot ()

