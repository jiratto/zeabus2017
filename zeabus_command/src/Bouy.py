#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, String
from AIControl import AIControl
from zeabus_vision_srv_msg.srv import vision_srv_bouy
from zeabus_vision_srv_msg.msg import vision_msg_bouy

class Bouy (object):

	def __init__ (self):
		print "Start Mission Bouy"

		rospy.init_node ('bouy_node')
		self.aicontrol = AIControl ()
		self.ball = 0
		self.data = None
		self.leftColor = None
		self.state = 1
		self.time = 0.5
		self.startPos = None ## First point that you see three balls
		self.destPos = None ## Position that you hit ball
		self.command = rospy.Publisher ('/zeabus/cmd_vel', Twist, queue_size=10)
		self.turn_yaw_rel = rospy.Publisher ('/fix/rel/yaw', Float64, queue_size=10)
		self.distance = []

		bouy_srv = 'vision_bouy'
		rospy.wait_for_service (bouy_srv)
		self.detect_bouy = rospy.ServiceProxy (bouy_srv, vision_srv_bouy)

	def find_num (self):
		self.data = self.detect_bouy (String ('bouy'), String ('a'))
		self.data = self.data.data

		return self.data.num
	
	def movement (self, color):
		count = 0
		x = []
		y = []
		p = []
		area = []

		xAvr = 0
		yAvr = 0
		areaAvr = 0

		for i in xrange (10):
			self.data = self.detect_bouy (String ('bouy'), String (color))
			self.data = self.data.data
			if self.data.appear:
				x.append (self.data.cx[0])
				y.append (self.data.cy[0])
				p.append (self.data.prob[0])
				area.append (self.data.area[0])
				count += 1
			rospy.sleep (0.1)

		if count != 0:
			xImg = self.aicontrol.average (x)
			yImg = self.aicontrol.average (y)
			prob = self.aicontrol.average (p)
			areaImg = self.aicontrol.average (area)
		else:
			print 'NOT FOUND'

			self.aicontrol.drive_xaxis (1)
			rospy.sleep (2)

			return False
		# self.time = self.data.area[0]

		print ('xImg: ', xImg)
		print ('yImg: ', yImg)
		print ('prob: ', prob)
		print ('area: ', areaImg)
		print ('appear: yes')
		print ('-----------------')

		if self.aicontrol.is_center ([xImg, yImg], -0.04, 0.04, -0.05, 0.05):
			print 'CENTER'
			tempDist = []

			print areaImg
			## check ratio area before hit_and_back ##
			while areaImg < 0.005 and not rospy.is_shutdown ():
				print 'IN LOOP'

				x = []
				area = []
				count = 0

				for i in xrange (10):
					self.data = self.detect_bouy (String ('bouy'), String (color))
					self.data = self.data.data
					if self.data.appear:
						x.append (self.data.cx[0])
						area.append (self.data.area[0])
						count += 1
					rospy.sleep (0.1)

				if count != 0:
					xImg = self.aicontrol.average (x)
					areaImg = self.aicontrol.average (area)
				else:
					print 'NOT FOUND'

					self.aicontrol.drive_xaxis (1)
					rospy.sleep (2)
					continue
				
				print ('xImg: ', xImg)
				print ('area: ', areaImg)

				if -0.05 <= xImg <= 0.05:
					print 'Y-AXIS CENTER'

					if areaImg <= 0:
						vx = 1
					else:
						vx = 1 / areaImg
					vx = self.aicontrol.adjust (vx, -0.8, -0.3, 0.3, 0.8)
					self.aicontrol.drive_xaxis (vx)
					rospy.sleep (self.time)
				else:
					print 'ADJUST Y-AXIS'
					vy = self.aicontrol.adjust (xImg, -0.6, -0.3, 0.3, 0.6)
					self.aicontrol.drive_xaxis (vx)
					rospy.sleep (self.time)
				self.aicontrol.stop (1)
				
			print 'HIT AND BACK'

			self.hit_and_back ()

			## trackback backward ##
			# for i in xrange (len (tempDist)):
			# 	self.aicontrol.drive_xaxis (-data[i])
			# 	rospy.sleep (self.time)

			# self.aicontrol.stop (1)

			# print 'TRACKBACK'
			# self.aicontrol.trackback (self.distance, self.time)
			# self.distance = []
			self.destPos = self.aicontrol.get_position ()
			
			return True
		else:
			print 'NOT CENTER'

			pos = self.aicontrol.get_position ()

			if -0.05 <= pos[2] <= 0.05:
				print 'FIX Z'

				vy = self.aicontrol.adjust (xImg, -0.6, -0.3, 0.3, 0.6)
				self.distance.append (vy)
				self.aicontrol.drive_yaxis (vy)
				rospy.sleep (self.time)
				self.aicontrol.stop (0.5)
			else:
				print 'MOVE YZ'
				vy = self.aicontrol.adjust (xImg, -0.6, -0.3, 0.3, 0.6)
				vz = self.aicontrol.adjust (yImg, -1, -0.95, 0.005, 0.01)
				
				# self.distance.append (vy)
				# self.distance.append (vz)

				self.aicontrol.drive_zaxis (vz)
				rospy.sleep (self.time)
				self.aicontrol.stop (1)
				
				self.aicontrol.drive_yaxis (-vy)
				rospy.sleep (self.time)
				self.aicontrol.stop (1)
			
			return False 

	def hit_and_back (self):
		self.aicontrol.drive_xaxis (1)
		rospy.sleep (8)
		self.aicontrol.stop (1)
		
		print 'HIT BALL!'

		self.aicontrol.drive_xaxis (-1)
		rospy.sleep (8)
		self.aicontrol.stop (1)

		# self.aicontrol.trackback (self.distance, self.time)
		# self.distance = []

	def one_ball (self):
		red_data = self.detect_bouy (String ('bouy'), String ('red'))
		red_data = red_data.data

		yellow_data = self.detect_bouy (String ('bouy'), String ('yellow'))
		yellow_data = yellow_data.data

		green_data = self.detect_bouy (String ('bouy'), String ('green'))
		green_data = green_data.data

		if red_data.appear:
			while not yellow_data.appear and not red_data.appear:
				red_data = self.detect_bouy (String ('bouy'), String ('red'))
				red_data = red_data.data

				yellow_data = self.detect_bouy (String ('bouy'), String ('yellow'))
				yellow_data = yellow_data.data

				##### SLIDE TO FIND YELLOW
			self.two_ball ()
		elif green_data.appear:
			while not yellow_data.appear and not green_data.appear:
				green_data = self.detect_bouy (String ('bouy'), String ('green'))
				green_data = green_data.data

				yellow_data = self.detect_bouy (String ('bouy'), String ('yellow'))
				yellow_data = yellow_data.data

				##### SLIDE TO FIND YELLOW
			self.two_ball ()
		elif yellow_data.appear:
			while not yellow_data.appear and not green_data.appear and not red_data.appear:
				green_data = self.detect_bouy (String ('bouy'), String ('green'))
				green_data = green_data.data

				yellow_data = self.detect_bouy (String ('bouy'), String ('yellow'))
				yellow_data = yellow_data.data

				red_data = self.detect_bouy (String ('bouy'), String ('red'))
				red_data = red_data.data

				##### BACKWARD TO SEE THREE BALL
			self.three_ball ()

	def two_ball (self):
		red_data = self.detect_bouy (String ('bouy'), String ('red'))
		red_data = red_data.data

		yellow_data = self.detect_bouy (String ('bouy'), String ('yellow'))
		yellow_data = yellow_data.data

		green_data = self.detect_bouy (String ('bouy'), String ('green'))
		green_data = green_data.data

		if red_data.appear and yellow_data.appear and not green_data.appear:
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
				
				if green_data.data.appear:
					break

			check = False
			while not rospy.is_shutdown ():
				check = self.movement ('green')
				if check:
					break

		elif not red_data.appear and yellow_data.appear and green_data.appear:
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
				
				if red_data.data.appear:
					break

			check = False
			while not rospy.is_shutdown ():
				check = self.movement ('red')
				if check:
					break

	def three_ball (self):
		# check = False
		color = 'r'
		# while not rospy.is_shutdown ():
		# 	if self.movement ('r'):
				

		# self.hit_and_back ()

	def comeback (self):
		start_x = self.startPos[0]
		start_y = self.startPos[1]
		start_z = self.startPos[2]
		dest_x = self.destPos[0]
		dest_y = self.destPos[1]
		dest_z = self.destPos[2]

		## comeback X
		print 'COMEBACK X'
		diff_x = -(dest_x - start_x)
		self.aicontrol.drive_x_rel (diff_x)

		## comeback Y
		print 'COMEBACK Y'
		diff_y = -(dest_y - start_y)
		self.aicontrol.drive_y_rel (diff_y)

		## comeback Z
		print 'COMEBACK Z'
		self.aicontrol.fix_zaxis (start_z)
		

	def run (self):
		while self.find_num () <= 0:
			self.aicontrol.drive ([1, 0, 0, 0, 0, 0])
			rospy.sleep (0.5)
			self.aicontrol.stop (0.1)

		num = self.find_num ()
		print ('Found %d balls', num)

		if num == 3:
			three_ball ('red')
			three_ball ('green')
			self.aicontrol.drive_xaxis (1)
			rospy.sleep (5)

		elif num == 2:
			two_ball ()

		elif num == 1:
			one_ball ()

	def get_data (self, color):
		while not rospy.is_shutdown ():
			count = 0
			x = []
			y = []
			p = []
			area = []

			xAvr = 0
			yAvr = 0
			areaAvr = 0

			for i in xrange (10):
				self.data = self.detect_bouy (String ('bouy'), String (color))
				self.data = self.data.data
				if self.data.appear:
					x.append (self.data.cx[0])
					y.append (self.data.cy[0])
					p.append (self.data.prob[0])
					area.append (self.data.area[0])
					count += 1
				rospy.sleep (0.1)

			if count != 0:
				xImg = self.aicontrol.average (x)
				yImg = self.aicontrol.average (y)
				prob = self.aicontrol.average (p)
				areaImg = self.aicontrol.average (area)
			else:
				print 'NOT FOUND'

				self.aicontrol.drive_xaxis (1)
				rospy.sleep (2)

				return False
			# self.time = self.data.area[0]

			count = 0

			print ('xImg: ', xImg)
			print ('yImg: ', yImg)
			print ('prob: ', prob)
			print ('area: ', areaImg)
			print ('appear: yes')
			print ('-----------------')
			rospy.sleep (1)

	def test_move (self, color):
		self.startPos = self.aicontrol.get_position ()
		self.aicontrol.fix_zaxis (-2.6)
		while not self.movement (color) and not rospy.is_shutdown ():
			print 'not pass'
		print 'pass'
		self.comeback ()
		print 'COMPLETE'
		self.aicontrol.stop (1)

if __name__ == '__main__':
	bouy = Bouy ()
	# Bouy.run ()
	bouy.find_num ()
	# bouy.movement ('y')
	# bouy.get_data ('y')
	bouy.test_move ('y')