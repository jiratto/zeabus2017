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
		self.countStateOne = 30
		self.countStateTwo = 20
		self.countStateThree = 20
		self.task = 'navigate'
		self.state = 1 # state go to navigate
		self.data = None
		self.angleBot = None

		navigate_srv = 'vision_navigate'
		rospy.wait_for_service (navigate_srv)
		self.detectNav = rospy.ServiceProxy (navigate_srv, vision_srv_navigate) 

	## state 1
	def go_to_navigate (self):
		zero = 0
		one = 0
		two = 0
		mul = 0
		cx = [0] * 3
		cy = [0] * 3
		area = [0] * 3

		check = 0

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
				vx = (1 / area[2]) * 2
				self.aicontrol.drive_xaxis (vx)
				rospy.sleep (5)
			else:
				print 'Drive to center'
				vy = self.aicontrol.adjust (avrCx, -0.6, -0.4, 0.4, 0.6)
				self.aicontrol.drive ([0, -vy, 0, 0, 0, 0])
				rospy.sleep (3)
		elif one > two >= zero:
			print 'ONE LEG'

			if self.aicontrol.is_fail (self.countState1):
				print 'STATE 1 FAIL'
				self.state = 0
				
			if mul > 0:
				vy = 0.4
			elif mul < 0:
				vy = -0.4
			else:
				return
			self.aicontrol.drive_yaxis (vy)
			rospy.sleep (2)
			self.countStateOne -= 1
		elif zero > one >= two:
			print 'ZERO'
			self.aicontrol.drive_xaxis (1)
			rospy.sleep (1)
			check += 1
			if check >= 3:
				self.aicontrol.fix_zaxis (depth.NAVIGATE_ROLL)
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
			
			angle.append (self.data.angle)
			if self.data.appear:
				found += 1
			else:
				notFound += 1

		if found > notFound:
			print 'FOUND'
			self.angleBot = self.aicontrol.average (angle)
			print ('angle: ', self.angleBot)

			print ('State 3 : back and roll')
			self.state = 3
		else:
			print 'NOT FOUND'
			self.aicontrol.drive_xaxis (0.8)
			rospy.sleep (3)

			self.countStateTwo -= 1

			if self.aicontrol.is_fail (self.countStateTwo):
				print ('STATE 2_FAIL')
				self.state = 0

	## state 3
	def back_and_roll (self):
		print 'BACK AND TURN'
		self.aicontrol.drive_xaxis (-0.8)
		rospy.sleep (5)
		self.aicontrol.turn_yaw_relative (self.angleBot)
		rospy.sleep (2)

		zero = 0
		one = 0
		two = 0
		mul = 0
		cx = [0] * 3
		cy = [0] * 3
		area = [0] * 3

		check = 0
		countFail = 10

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
				vx = (1 / area[2]) * 2
				self.aicontrol.drive_xaxis (vx)
				rospy.sleep (5)
			else:
				print 'Drive to center'
				vy = self.aicontrol.adjust (avrCx, -0.6, -0.4, 0.4, 0.6)
				self.aicontrol.drive_yaxis (-vy)
				rospy.sleep (3)
		elif one > two >= zero:
			print 'ONE LEG'

			if mul > 0:
				vy = 0.4
			elif mul < 0:
				vy = -0.4
			else:
				return
			self.aicontrol.drive_yaxis (vy)
			rospy.sleep (2)
			self.countStateThree -= 1

			if self.aicontrol.is_fail (self.countStateThree):
				print 'STATE 3 FAIL'
				self.state = 0

		elif zero > one >= two:
			print 'ZERO'
			self.aicontrol.drive_xaxis (1)
			rospy.sleep (1)
			check += 1
			if check >= 3:
				self.aicontrol.drive_xaxis (1)
				rospy.sleep (10)
				self.aicontrol.stop (1)
				
				print 'State 4 : Navigate complete'
				self.state = 4

	def run (self):
		task = 'navigate'
		cam = 'top'

		self.aicontrol.fix_zaxis (depth.NAVIGATE_DETECTING)
		self.aicontrol.drive_xaxis (1)
		rospy.sleep (5)

		while not rospy.is_shutdown ():
			if self.state == 1:
				self.go_to_navigate ()
			elif self.state == 2:
				self.check_bottom ()
			elif self.state == 3:
				self.back_and_roll ()
			elif self.state == 4 or self.state == 0:
				self.aicontrol.fix_zaxis (depth.FLOATING)
				break
			self.aicontrol.stop (0.5)

if __name__ == '__main__':
	navigate = Navigate ()
	navigate.run ()

