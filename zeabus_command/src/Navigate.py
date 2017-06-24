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

		check = 0
		self.aicontrol.fix_zaxis (-3.3)

		while not rospy.is_shutdown ():
			self.aicontrol.stop (1)
			zero = 0
			one = 0
			two = 0
			mul = 0
			cx = [0] * 3
			cy = [0] * 3
			area = [0] * 3

			for i in xrange (10):
				self.data = self.detectNav (String (mission), String (req))
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
				if self.aicontrol.is_center ([cx[2]/two, 0], -0.15, 0.15, -0.1, 0.1):
					print 'Center !!'
					self.aicontrol.drive_xaxis (1/area[2] * 2)
				else:
					print 'Drive to center'
					vx = self.aicontrol.adjust (cx[2]/two * 10, -0.6, -0.2, 0.2, 0.6)
					vy = self.aicontrol.adjust (cy[2]/two, -0.6, -0.2, 0.2, 0.6)
					self.aicontrol.drive ([0, -vx, 0, 0, 0, 0])
				rospy.sleep (3)
			elif one > two >= zero:
				print 'ONE LEG'
				self.aicontrol.drive_yaxis (0.4 * mul/5)
				rospy.sleep (3)
			elif zero > one >= two:
				print 'ZERO'
				self.aicontrol.fix_zaxis (-2.5)
				self.aicontrol.drive_xaxis (1)
				rospy.sleep (1)
				check += 1
				if check >= 3:
					self.aicontrol.roll (2)
					print 'MISSION COMPLETE'
					break
		
		self.aicontrol.stop (1)

if __name__ == '__main__':
	navigate = Navigate ()
	navigate.run ()