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
		self.time = None
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
	
	def movement (self, color):
		self.data = self.detect_bouy (String ('bouy'), String (color))
		self.data = self.data.data
		xImg = self.data.x[0]
		yImg = self.data.y[0]

		self.time = self.data.area[0]

		if self.aicontrol.is_center ([xImg, yImg], -0.05, -0.05, -0.05, 0.05):
			print 'Target at Center'

		vx = self.aicontrol.adjust (xImg, -0.6, -0.2, 0.2, 0.6)
		vy = self.aicontrol.adjust (yImg, -0.6, -0.2, 0.2, 0.6)
			
		self.distance.append (vx)
		self.distance.append (vy)

		self.aicontrol.drive ([0, vx, 0, 0, 0, 0])
		rospy.sleep (self.time)
		self.aicontrol.stop (0.1)

		self.aicontrol.drive ([0, 0, vy, 0, 0, 0])
		rospy.sleep (self.time)
		self.aicontrol.stop (0.1)

		return self.aicontrol.is_center ([xImg, yImg], -0.05, -0.05, -0.05, 0.05) 

	def hit_and_back (self):
		self.aicontrol.drive_xaxis (1)
		rospy.sleep (5)
		self.aicontrol.stop (1)
		
		self.aicontrol.drive_xaxis (-1)
		rospy.sleep (5)
		self.aicontrol.stop (1)

		self.aicontrol.trackback (self.distance, self.time)
		self.distance = []

	def two_ball (self):
		red_data = self.detect_bouy (String ('bouy'), String ('red'))
		red_data = red_data.data

		yellow_data = self.detect_bouy (String ('bouy'), String ('yellow'))
		yellow_data = yellow_data.data

		green_data = self.detect_bouy (String ('bouy'), String ('green'))
		green_data = green_data.data

		if red_data.found[0] and yellow_data.found[0] and not green_data.found[0]:
			check = False
			while not rospy.is_shutdown ():
				check = self.movement ('red')
				if check:
					break
			self.hit_and_back ()

			check = False
			while not rospy.is_shutdown ():
				check = self.movement ('yellow')
				if check:
					break
			self.hit_and_back ()

			### slide to green_ball
			###
			###
			###
			###
			###

			while rospy.is_shutdown ():
				green_data = self.detect_bouy (String ('bouy'), String ('green'))
				green_data = green_data.data				
				
				if green_data.data.found[0]:
					break

			check = False
			while not rospy.is_shutdown ():
				check = self.movement ('green')
				if check:
					break

		else if not red_data.found[0] and yellow_data.found[0] and green_data.found[0]:
			check = False
			while not rospy.is_shutdown ():
				check = self.movement ('yellow')
				if check:
					break
			self.hit_and_back ()

			check = False
			while not rospy.is_shutdown ():
				check = self.movement ('green')
				if check:
					break
			self.hit_and_back ()

			### slide to red_ball
			###
			###
			###
			###
			###

			while rospy.is_shutdown ():
				red_data = self.detect_bouy (String ('bouy'), String ('red'))
				red_data = red_data.data				
				
				if red_data.data.found[0]:
					break

			check = False
			while not rospy.is_shutdown ():
				check = self.movement ('red')
				if check:
					break

	def three_ball (self, color):
		self.time = 0.5

		check = False
		while not rospy.is_shutdown ():
			check = self.movement (color)
			if check:
				break

		self.hit_and_back ()

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
			self.aicontrol.drive_xaxis (1)
			rospy.sleep (5)

		elif num == 2:
			two_ball ()

		elif num == 1: