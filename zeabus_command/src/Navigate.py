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
		self.data = None

		navigate_srv = 'vision_navigate'
		rospy.wait_for_service (navigate_srv)
		self.detectNav = rospy.ServiceProxy (navigate_srv, vision_srv_navigate) 

	def run (self):
		task = 'navigate'
		cam = 'top'

		## check zero 3 times before roll ##
		check = 0

		isFail = 34

		self.aicontrol.fix_zaxis (depth.NAVIGATE_DETECTING)
		self.aicontrol.drive_xaxis (1)
		rospy.sleep (5)

		while not rospy.is_shutdown ():
			self.aicontrol.stop (1)
			
			## task: top ##
			zero = 0
			one = 0
			two = 0
			mul = 0
			cx = [0] * 3
			cy = [0] * 3
			area = [0] * 3

			## task: bot ##
			found = 0
			notFound = 0
			angle = []

			for i in xrange (10):
				self.data = self.detectNav (String (task), String (cam))
				self.data = self.data.data
				if cam == 'top':
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
				elif cam == 'bot':
					angle.append (self.data.angle)
					if self.data.appear:
						found += 1
					else:
						notFound += 1

			print ('cx[2]: ', cx[2])
			print ('cy[2]: ', cy[2])
			print ('area[2]: ', area[2])
			print ('cx[1]: ', cx[1])
			print ('cy[1]: ', cy[1])

			if cam == 'top':
				if two > one >= zero:
					print 'TWO LEGS'
					if self.aicontrol.is_center ([cx[2]/two, 0], -0.1, 0.1, -0.1, 0.1):
						print 'Center !!'
						self.aicontrol.drive_xaxis (1/area[2] * 2)
						rospy.sleep (5)
					else:
						print 'Drive to center'
						vy = self.aicontrol.adjust (cx[2]/two, -0.6, -0.4, 0.4, 0.6)
						self.aicontrol.drive ([0, -vy, 0, 0, 0, 0])
						rospy.sleep (3)
				elif one > two >= zero:
					print 'ONE LEG'
					print ('isFail: ', isFail)

					if self.aicontrol.is_fail (isFail):
						self.aicontrol.fix_zaxis (depth.NAVIGATE_ROLL)
						self.aicontrol.stop (1)
						cam = 'bot'
						# self.aicontrol.drive_xaxis (1)
						# rospy.sleep (18)
						# self.aicontrol.roll (2)
						print 'SWAP TO BOTTOM CAMERA'
					if mul > 0:
						vy = 0.4
					elif mul < 0:
						vy = -0.4
					else:
						continue
					self.aicontrol.drive_yaxis (vy)
					rospy.sleep (2)
					# if isFail % 7 == 0:
					# 	self.aicontrol.drive_xaxis (1)
					# 	rospy.sleep (2)

					isFail -= 1
				elif zero > one >= two:
					print 'ZERO'
					self.aicontrol.drive_xaxis (1)
					rospy.sleep (1)
					check += 1
					if check >= 3:
						self.aicontrol.fix_zaxis (depth.NAVIGATE_ROLL)
						self.aicontrol.stop (1)
						cam = 'bot'
						# self.aicontrol.drive_xaxis (1)
						# rospy.sleep (18)
						# self.aicontrol.roll (2)
						print 'SWAP TO BOTTOM CAMERA'
			elif cam == 'bot':
				if found > notFound:
					print 'FOUND'
					angleAvr = self.aicontrol.average (angle)
					# print ('angle: ', angleAvr)
					# self.aicontrol.turn_yaw_relative (angleAvr)
					# rospy.sleep (2)

					print 'FOUND AND BACK'
					self.aicontrol.drive_xaxis (-0.7)
					rospy.sleep (3)
					# self.aicontrol.drive_yaxis (1)
					# rospy.sleep (2)
					self.aicontrol.drive_xaxis (1)
					rospy.sleep (2)
					self.aicontrol.roll (2)
					# self.aicontrol.drive_xaxis (0.8)
					# rospy.sleep (10)
					self.aicontrol.fix_zaxis (-0.2)
					break
				else:
					print 'NOT FOUND'
					self.aicontrol.drive_xaxis (0.8)
					rospy.sleep (3)
		
		self.aicontrol.stop (1)

if __name__ == '__main__':
	navigate = Navigate ()
	navigate.run ()
