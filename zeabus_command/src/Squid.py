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

class Squid (object):
	def __init__ (self):
		rospy.init_node ('squid_node_init')
		print 'init node squid complete'

		self.aicontrol = AIControl ()
		self.hardware = Hardware ()

		print 'wait vision squid'
		rospy.wait_for_service ('vision_squid')
		self.detect_sq = rospy.ServiceProxy ('vision_squid', vision_srv_default)
		self.isFail = 30

		print 'Prepare to do task Squid'

	def run (self):
		while not rospy.is_shutdown () and not self.aicontrol.is_fail (self.isFail):
			print 'Loop of detecting'
			ax = []
			ay = []
			aarea = []
			count = 0
			found = True

			print 'Collect avg'
			for i in xrange (10):
				self.data = self.detect_sq (String ('squid'), String ('s'))
				self.data = self.data.data

				if self.data.appear:
					ax.append (self.data.x)
					ay.append (self.data.y)
					aarea.append (self.data.area)
					count += 1
				rospy.sleep (0.2)

			if count == 0:
				found = False

			if found:
				self.aicontrol.stop (1)

				print 'Found Small Squid'
				x = self.aicontrol.average (ax)
				y = self.aicontrol.average (ay)
				area = self.aicontrol.average (aarea)

				if self.aicontrol.is_center ([x, y], -0.1, 0.1, -0.2, 0.2):
					self.aicontrol.stop (1)
					if area > 0.1:
						print 'Center Squid'
						self.aicontrol.stop (2)
						for i in xrange (3):
							self.hardware.command ('fire_right', 'fire')
							rospy.sleep (1)
						# self.aicontrol.stop (1)
						break
					else:
						print 'Center but not near'
						self.aicontrol.drive_xaxis (0.5)
						rospy.sleep (0.5)
						# self.aicontrol.stop (0.5)
				else:
					print ax
					print x
					print 'Not Center'
					y = self.aicontrol.adjust (x, -0.3, -0.1, 0.1, 0.3)
					z = self.aicontrol.adjust (y, -0.5, -0.3, 0.0, 0.0)
					if area < 0.2:
						print 'Adjust'
						self.aicontrol.drive ([0.1, y, z, 0, 0, 0])
						rospy.sleep (0.2)
					else:
						print 'Forward'
						self.aicontrol.drive ([0.1, 0, 0, 0, 0, 0])
						rospy.sleep (0.1)
			else:
				self.aicontrol.drive_xaxis (0.2)
				rospy.sleep (0.3)
				self.isFail -= 1


	# def find_all (self):
	# 	self.aicontrol.fix_zaxis (dep)
	# 	while not rospy.is_shutdown ():
	# 		self.data = self.detect_sq (String ('squid'), String ('a'))
	# 		self.data = self.data.data

	# 		self.sm = self.detect_sq (String ('squid'), String ('s'))
	# 		self.sm = self.sm.data

	# 		self.lg = self.detect_sq (String ('squid'), String ('b'))
	# 		self.lg = self.lg.data

	# 		if self.lg.appear and self.sm.appear:
	# 			return True
	# 		else:


if __name__ == '__main__':
	squid = Squid ()
	squid.run ()
	print 'FINISH'