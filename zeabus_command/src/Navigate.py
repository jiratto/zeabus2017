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
		mission = 'navigate'
		req = 'Yeah'

		self.aicontrol.fix_zaxis (-3)

		while not rospy.is_shutdown ():
			self.aicontrol.stop (1)

			zero = 0
			one = 0
			two = 0
			mul = 0
			cx = list()
			cy = list()
			area = list()

			for i in xrange (10):
				self.data = self.detectNav (String (mission), String (req))
				self.data = self.data.data

				if self.data.numVertical >= 2:
					two += 1
					cx[2] += self.data.cx
					cy[2] += self.data.cy
					area[2] += self.data.area
				elif self.data.numVertical == 1:
					one += 1
					area[1] += self.data.area
					mul += self.data.direction
				elif self.data.numVertical == 0:
					zero += 1

			if two > one >= zero:
				print 'TWO LEGS'
				if self.aicontrol.is_center ([cx[2]/two, cy[2]/two], -0.2, 0.2, -0.1, 0.1):
					print 'Center !!'
					self.aicontrol.drive_xaxis (1/area[2]/10)
				else:
					print 'Drive to center'
					vx = self.aicontrol.adjust (cx[2]/two, -0.5, -0.1, 0.1, 0.5)
					vy = self.aicontrol.adjust (cy[2]/two, -0.5, -0.1, 0.1, 0.5)
					self.aicontrol.drive ([0, vx, vy, 0, 0, 0])
				rospy.sleep (0.5)
			elif one > two >= zero:
				print 'ONE LEG'
				self.aicontrol.drive_yaxis (0.3 * mul)
				rospy.sleep (0.5)
			elif zero > one >= two:
				print 'Ja roll la na'
				self.aicontrol.drive_xaxis (1)
				rospy.sleep (1)
				self.aicontrol.roll (2)
				print 'MISSION COMPLETE'
				break
			self.aicontrol.stop (1)

if __name__ == '__main__':
	navigate = Navigate ()
	navigate.run ()