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

		print 