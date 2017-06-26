#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, String
from zeabus_vision_srv_msg.msg import vision_msg_navigate
from zeabus_vision_srv_msg.srv import vision_srv_navigate
from AIControl import AIControl

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

		## check zero 3 times before role ##
		check = 0

		countOneLeg = 0

		self.aicontrol.fix_zaxis (-3.6)
		self.aicontrol.drive_xaxis (1)
		rospy.sleep (4)

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


			if cam == 'top':
				if mul > 0:
					mul = 1
				elif mul <= 0:
					mul = -1

				if two > one >= zero:
					print 'TWO LEGS'
					if self.aicontrol.is_center ([cx[2]/two, 0], -0.2, 0.1, -0.1, 0.1):
						print 'Center !!'
						self.aicontrol.drive_xaxis (1/area[2] * 2)
						rospy.sleep (5)
					else:
						print 'Drive to center'
						vy = self.aicontrol.adjust (cx[2]/two, -0.6, -0.2, 0.2, 0.6)
						self.aicontrol.drive ([0, -vy, 0, 0, 0, 0])
						rospy.sleep (3)
				elif one > two >= zero:
					print 'ONE LEG'
					if mul >= 0:
						vy = 0.3
					else:
						vy = -0.15
					self.aicontrol.drive_yaxis (vy)
					rospy.sleep (2)
				elif zero > one >= two:
					print 'ZERO'
					self.aicontrol.drive_xaxis (1)
					rospy.sleep (1)
					check += 1
					if check >= 3:
						self.aicontrol.fix_zaxis (-2.5)
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
					print ('angle: ', angleAvr)
					self.aicontrol.turn_yaw_relative (angleAvr)
					rospy.sleep (2)
					# self.aicontrol.roll (2)
					self.aicontrol.drive_xaxis (1)
					rospy.sleep (8)
					self.aicontrol.fix_zaxis (-0.2)
					break
				else:
					print 'NOT FOUND'
					self.aicontrol.drive_xaxis (1)
					rospy.sleep (2)
		
		self.aicontrol.stop (1)

if __name__ == '__main__':
	navigate = Navigate ()
	navigate.run ()
