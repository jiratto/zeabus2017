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
        self.distance = []

        # bouy_srv = 'bouy'
        # rospy.wait_for_service (bouy_srv)
        # self.detect_bouy = rospy.ServiceProxy (bouy_srv, bouy_sim)

	def find_num (self):
		self.data = self.detect_bouy (String ('bouy'), String ('all'))
		self.data = self.data.data

		return self.data.num
	
	def three_ball (self, color):
		time = 0.5

		while not rospy.is_shutdown ():
			self.data = self.detect_bouy (String ('bouy'), String (color))
			self.data = self.data.data
			xImg = self.data.x[0]
			yImg = self.data.y[0]

			if self.aicontrol.is_center ([xImg, yImg], -0.05, -0.05, -0.05, 0.05):
				print 'Target at Center'
				break

			vx = self.aicontrol.adjust (xImg, -0.6, -0.2, 0.2, 0.6)
			vy = self.aicontrol.adjust (yImg, -0.6, -0.2, 0.2, 0.6)
			
			self.distance.append (vx)
			self.distance.append (vy)

			self.aicontrol.drive ([0, vx, 0, 0, 0, 0])
			rospy.sleep (time)
			self.aicontrol.stop (0.1)

			self.aicontrol.drive ([0, 0, vy, 0, 0, 0])
			rospy.sleep (time)
			self.aicontrol.stop (0.1)

		self.aicontrol.drive ([1, 0, 0, 0, 0, 0])
		rospy.sleep (5)
		self.aicontrol.stop (0.1)

		## trackback path ##
		self.aicontrol.drive ([-1, 0, 0, 0, 0, 0])
		rospy.sleep (5)
		self.aicontrol.stop (2)
		self.aicontrol.trackback (self.distance, time)
		
		self.distance = [] # clear array

	def run (self):
		while find_num () <= 0:
			self.aicontrol.drive ([1, 0, 0, 0, 0, 0])
			rospy.sleep (0.5)
			self.aicontrol.stop (0.1)

		num = find_num ()
		print ('Found %d balls', num)

		if num == 3:
			three_ball ('red')
			three_ball ('green')

		elif num == 2:

		elif num == 1: