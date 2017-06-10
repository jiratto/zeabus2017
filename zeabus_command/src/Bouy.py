#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, String
from AIControl import AIControl

class Bouy (object):
	
	def __init__ (self):
		print "Start Mission Bouy"

		rospy.init_node ('bouy_node')

		self.aicontrol = AIControl ()
		self.ball = 0
		self.data = None
		self.leftColor = None
		self.state = 1
		self.command = rospy.Publisher ('/cmd_vel', Twist, queue_size=10)
		self.turn_yaw_rel = rospy.Publisher ('/fix/rel/yaw', Float64, queue_size=10)
		
		# bouy_srv = 'bouy'
		# rospy.wait_for_service (bouy_srv)
		# self.detect_bouy = rospy.ServiceProxy (bouy_srv, bouy_sim)
	
	def find (self):
		bouy_data = self.detect_bouy