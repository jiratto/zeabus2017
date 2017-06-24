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

		while not rospy.is_shutdown ():
			self.data = self.detectNav (String (mission), String (req))
			self.data = self.data.data

			## get data ##
			cx = self.data.cx
			cy = self.data.cy
			verticalX = self.data.verticalX
			verticalY = self.data.verticalY
			area = self.data.ratioArea
			numVertical = self.data.numVertical

			print ('numVertical: ', numVertical)
			print ('cx: ', cx)
			print ('cy: ', cy)
			print ('verticalX: ', verticalX)
			print ('verticalY: ', verticalY)
			print ('ratio area: ', area)

			if numVertical >= 2:
				if self.aicontrol.is_center ([cx, cy], -0.2, 0.2, -0.1, 0.1):
					if area < 0.4:
						self.aicontrol.drive.xaxis (0.7)
					elif area < 0.7:
						self.aicontrol.drive.xaxis (0.3)
					elif area < 1:
						self.aicontrol.drive.xaxis (0.2)
					else:
						self.aicontrol.drive.xaxis (0.1)
				else:
					vx = self.aicontrol.adjust (cx, -0.5, -0.1, 0.1, 0.5)
					vy = self.aicontrol.adjust (cy, -0.5, -0.1, 0.1, 0.5)
					self.aicontrol.drive ([0, vx, 0, 0, 0, 0])
				rospy.sleep (2)
			elif numVertical == 1:
				while not verticalX == -999 and not verticalY == -999:
					vx = self.aicontrol.adjust (verticalX, -0.5, -0.1, 0.1, 0.5)
					vy = self.aicontrol.adjust (verticalY, -0.5, -0.1, 0.1, 0.5)
					self.aicontrol.drive ([0, vx, 0, 0, 0, 0])
					rospy.sleep (2)
			elif numVertical == 0:
				print "MISSION COMPLETE"
				break
			self.aicontrol.stop (1)

if __name__ == '__main__':
	navigate = Navigate ()
	navigate.run ()