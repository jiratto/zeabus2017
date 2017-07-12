#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, String
from zeabus_vision_srv_msg.msg import vision_msg_default
from zeabus_vision_srv_msg.srv import vision_srv_default
from AIControl import AIControl
from Hardware import Hardware
import Depth as depth
import Direction as Direction

class Binn (object):
	def __init__ (self):
		print 'Start Bin Mission'

		rospy.init_node ('bin_node_init')

		self.aicontrol = AIControl ()
		self.hardware = Hardware ()

		print 'Wait bin'
		rospy.wait_for_service ('vision_bin')
		self.detect_bin = rospy.ServiceProxy ('vision_bin', vision_srv_default)
		print 'Get vision complete'

		self.isFail = 30

	def nocover (self):
		self.aicontrol.fix_zaxis (depth.BINN_DETECTING)
		prep = 0
		
		while not rospy.is_shutdown () and not self.aicontrol.is_fail (self.isFail):
			ax = []
			ay = []
			aarea = []
			aangle = []
			count = 0
			found = True

			print self.isFail
			for i in xrange (10):
				print 'Collect data to avg'
				self.data = self.detect_bin (String ('bin'), String ('nocover'))
				self.data = self.data.data

				if self.data.appear:
					ax.append (self.data.x)
					ay.append (self.data.y)
					aarea.append (self.data.area)
					aangle.append (self.data.angle)
					count += 1

				rospy.sleep (0.1)

			if count == 0:
				found = False

			if found:
				print 'Found Binn'		
				x = self.aicontrol.average (ax)
				y = self.aicontrol.average (ay)
				area = self.aicontrol.average (aarea)
				
				if self.aicontrol.average (aangle) < 0:
					angle = min(aangle)
				else:
					angle = max(aangle)

				if self.aicontrol.is_center ([x, y], -0.1, 0.1, -0.1, 0.1):
					print 'CENTER'
					self.aicontrol.stop (1)
					self.aicontrol.turn_yaw_relative (angle)
					self.aicontrol.stop (2)
					prep += 1
					if prep == 3:
						self.aicontrol.fix_zaxis (depth.BINN_FIRE)
						rospy.sleep (5)
						# self.aicontrol.drive_xaxis (1)
						# rospy.sleep (0.5)

						print 'FIRE LEFT'
						self.aicontrol.stop (2)
						# for i in xrange (3):
						# 	self.hardware.command ('drop_left', 'drop')
						# 	rospy.sleep (1)
						self.aicontrol.stop (5)

						print 'FIRE RIGHT'
						self.aicontrol.drive_yaxis (1)
						rospy.sleep (1.2)
						self.aicontrol.stop (2)
						# for i in xrange (3):
						# 	self.hardware.command ('drop_right', 'drop')
						# 	rospy.sleep (1)
						self.aicontrol.stop (1)

						# rospy.sleep (3)
						# self.hardware.command ('drop_left', 'close')
						# rospy.sleep (3)
						# self.hardware.command ('drop_right', 'close')
						# self.aicontrol.stop (1)
						break
				else:
					x = self.aicontrol.adjust (x, -0.5, -0.1, 0.1, 0.5)
					y = self.aicontrol.adjust (y, -0.5, -0.1, 0.1, 0.5)
					self.aicontrol.drive ([x, y, 0, 0, 0, 0])
					rospy.sleep (self.aicontrol.adjust (area, 0.2, 0.2, 0.2, 0.6))
					# self.aicontrol.turn_yaw_relative (angle)
					rospy.sleep (2)
					self.isFail -= 0.2
			else:
				self.aicontrol.drive_xaxis (0.5)
				rospy.sleep (1)
				self.isFail -= 1

	def run (self):
		print 'RUNNING'
		self.nocover ()

if __name__ == '__main__':
	binn = Binn ()
	binn.run ()
	print 'FINISH'
