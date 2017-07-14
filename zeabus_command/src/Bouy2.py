#!/usr/bin/env python

import rospy
import math
import Depth as depth
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, String
from AIControl import AIControl
from zeabus_vision_srv_msg.srv import vision_srv_bouy
from zeabus_vision_srv_msg.msg import vision_msg_bouy
# from zeabus_controller.srv import navigation_srv

class Bouy (object):

	def __init__ (self):
		print "Start Mission Bouy"

		rospy.init_node ('bouy_node')
		self.aicontrol = AIControl ()
		self.ball = 0
		self.data = None
		self.nav = None
		self.state = 1
		self.time = 1
		self.startPos = None ## First point that you see three balls
		self.startZ = None
		self.startY = None
		self.destPos = None ## Position that you hit ball
		self.destY = None
		self.command = rospy.Publisher ('/zeabus/cmd_vel', Twist, queue_size=10)
		self.turn_yaw_rel = rospy.Publisher ('/fix/rel/yaw', Float64, queue_size=10)

		bouy_srv = 'vision_bouy'
		nav_srv = 'navigation'
		rospy.wait_for_service (bouy_srv)
		self.detect_bouy = rospy.ServiceProxy (bouy_srv, vision_srv_bouy)
		# self.navigation = rospy.ServiceProxy (nav_srv, navigation_srv)

	def find_num (self):
		self.data = self.detect_bouy (String ('bouy'), String ('a'))
		self.data = self.data.data

		return self.data.num

	def yellow_to_center (self):
		print 'ADJUST YELLOW'

		self.data = self.detect_bouy (String ('bouy'), String ('a'))
		self.data = self.data.data

		xImg = self.data.cx[0])

		if xImg == -999:
			self.aicontrol.drive_xaxis (0.6)
			rospy.sleep (0.5)
			return

		print ('xImg: ', xImg)

		if -0.04 <= xImg <= 0.04:
			print 'CENTER'

			return True
		else:
			print 'MOVE Y'
			vy = self.aicontrol.adjust (xImg, -0.6, -0.3, 0.3, 0.6)
			
			self.aicontrol.drive_yaxis (vy)
			rospy.sleep (self.time)
			# self.aicontrol.stop (1)
			return False

	def see_yellow_ball (self):
		yellow_data = self.detect_bouy (String ('bouy'), String ('o'))
		yellow_data = yellow_data.data

		xImg = self.data.cx[0]

		if yellow_data.appear:
			while not 0.04 <= xImg <= 0.04:
				print 'MOVE Y'
				print ('xImg: ', xImg)

				vy = self.aicontrol.adjust (xImg, -0.6, -0.3, 0.3, 0.6)
				
				self.aicontrol.drive_yaxis (vy)
				rospy.sleep (self.time)
				# self.aicontrol.stop (1)
				yellow_data = self.detect_bouy (String ('bouy'), String ('o'))
				yellow_data = yellow_data.data

				xImg = self.data.cx[0]

			return True

		return False

	def see_all_balls (self):
		n = self.find_num ()
		if n < 3:
			return False
		else:
			return True
	
	def movement (self, color):
		count = 0
		# x = []
		# y = []
		# p = []
		# area = []

		xImg = 0
		yImg = 0
		areaImg = 0
		appear = None

		# for i in xrange (10):
		# 	self.data = self.detect_bouy (String ('bouy'), String (color))
		# 	self.data = self.data.data
		# 	if self.data.appear:
		# 		x.append (self.data.cx[0])
		# 		y.append (self.data.cy[0])
		# 		# p.append (self.data.prob[0])
		# 		area.append (self.data.area[0])
		# 		count += 1
		# 	rospy.sleep (0.1)

		## NOT AVERAGE
		self.data = self.detect_bouy (String ('bouy'), String (color))
		self.data = self.data.data

		appear = self.data.appear

		if appear:
			xImg = self.data.cx[0]
			yImg = self.data.cy[0]
			areaImg = self.data.area[0]
		else:
			print 'NOT FOUND'

			self.aicontrol.drive_xaxis (1)
			rospy.sleep (2)

			return False

		print ('xImg: ', xImg)
		print ('yImg: ', yImg)
		print ('area: ', areaImg)
		print ('appear: yes')
		print ('-----------------')

		if self.aicontrol.is_center ([xImg, yImg], -0.04, 0.01, -0.05, 0.05):
			print 'CENTER'
			self.aicontrol.stop (5)
			tempDist = []

			print areaImg
			## check ratio area before hit_and_back ##
			while areaImg < 0.012 and not rospy.is_shutdown ():
				print 'IN LOOP'

				x = []
				area = []
				count = 0

				# for i in xrange (10):
				# 	self.data = self.detect_bouy (String ('bouy'), String (color))
				# 	self.data = self.data.data
				# 	if self.data.appear:
				# 		x.append (self.data.cx[0])
				# 		area.append (self.data.area[0])
				# 		count += 1
				# 	rospy.sleep (0.1)

				# if count != 0:
				# 	xImg = self.aicontrol.average (x)
				# 	areaImg = self.aicontrol.average (area)
				# else:
				# 	print 'NOT FOUND'

				# 	self.aicontrol.drive_xaxis (1)
				# 	rospy.sleep (2)
				# 	continue

				## NOT AVERAGE
				self.data = self.detect_bouy (String ('bouy'), String (color))
				self.data = self.data.data

				appear = self.data.appear

				if appear:
					xImg = self.data.cx[0]
					yImg = self.data.cy[0]
					areaImg = self.data.area[0]
				else:
					print 'NOT FOUND'

					self.aicontrol.drive_xaxis (0.6)
					rospy.sleep (self.time)

					continue
				
				print ('xImg: ', xImg)
				print ('area: ', areaImg)

				# if -0.05 <= xImg <= 0.05:
				print 'Y-AXIS CENTER'

				if areaImg <= 0:
					vx = 1
				else:
					vx = (1 / areaImg) / 10
				vx = self.aicontrol.adjust (vx, -0.8, -0.3, 0.3, 0.8)
				self.aicontrol.drive_xaxis (vx)
				rospy.sleep (self.time)
				# else:
				# 	print 'ADJUST Y-AXIS'
				# 	vy = self.aicontrol.adjust (xImg, -0.6, -0.3, 0.3, 0.6)
				# 	self.aicontrol.drive_yaxis (vy)
				# 	rospy.sleep (self.time)
				# self.aicontrol.stop (1)
				
			print 'HIT AND BACK'
			self.aicontrol.stop (1)

			self.hit_and_back ()
			self.comeback ()
			
			return True
		else:
			print 'NOT CENTER'

			if -0.05 <= yImg <= 0.05:
				print 'FIX Z'

				vy = self.aicontrol.adjust (xImg, -0.6, -0.3, 0.3, 0.6)
				# self.distance.append (vy)
				self.aicontrol.drive_yaxis (vy)
				rospy.sleep (self.time)
				self.aicontrol.stop (0.5)
			else:
				print 'MOVE YZ'
				vy = self.aicontrol.adjust (xImg, -0.6, -0.3, 0.3, 0.6)
				vz = self.aicontrol.adjust (yImg, -1, -0.95, 0.05, 0.1)
				
				# self.distance.append (vy)
				# self.distance.append (vz)

				self.aicontrol.drive_zaxis (vz)
				rospy.sleep (self.time)
				# self.aicontrol.stop (1)
				
				self.aicontrol.drive_yaxis (vy)
				rospy.sleep (self.time)
				# self.aicontrol.stop (1)
			
			return False 

	def hit_and_back (self):
		self.aicontrol.drive_yaxis (0.4)
		rospy.sleep (2)

		self.aicontrol.drive_xaxis (1)
		rospy.sleep (10)
		self.aicontrol.stop (1)
		# self.aicontrol.drive_x_rel (1)

		print 'HIT BALL!'

		self.aicontrol.drive_xaxis (-1)
		rospy.sleep (14)
		self.aicontrol.stop (1)
		# self.aicontrol.drive_x_rel (-1)
		rospy.sleep (1)
		self.destPos = self.aicontrol.get_position ()
		self.destY = self.destPos[1]
		print self.destPos[1]
		rospy.sleep (5)

		# print self.destPos

	def comeback (self):
		print ('start_y: ', self.startY)
		print ('dest_y: ', self.destY)

		if self.destY > self.startY:
			dir = -1
		elif self.destY < self.startY:
			dir = 1
		else:
			dir = 0

		print dir
		
		# Slide to see yellow
		while not self.see_yellow_ball ():
			self.aicontrol.drive_yaxis (0.7 * dir)
			rospy.sleep (1)
			self.aicontrol.stop (0.5)
		# if dir == 1:
		# 	self.aicontrol.drive_yaxis (1)
		# 	rospy.sleep (8)
		# elif dir == -1:
		# 	self.aicontrol.drive_yaxis (-1)
		# 	rospy.sleep (7)

		## Go backward
		while not self.see_all_balls ():
			self.aicontrol.drive_xaxis (-1)
			rospy.sleep (0.5)
			self.aicontrol.stop (0.5)

		self.aicontrol.stop (0.5)
		
		print 'COMEBACK COMPLETE'

		# nav_data = self.navigation (Float64 (start_x), Float64 (start_y), Float64 (start_yaw))

		## comeback Z
		print 'COMEBACK Z'
		print self.startZ
		self.aicontrol.fix_zaxis (self.startZ)

	def one_ball (self):
		red_data = self.detect_bouy (String ('bouy'), String ('r'))
		red_data = red_data.data

		yellow_data = self.detect_bouy (String ('bouy'), String ('y'))
		yellow_data = yellow_data.data

		green_data = self.detect_bouy (String ('bouy'), String ('g'))
		green_data = green_data.data

		if red_data.appear:
			## Slide to yellow ball
			while not self.see_yellow_ball ():
				self.aicontrol.drive_yaxis (0.4)
				rospy.sleep (0.5)
		elif green_data.appear:
			## Slide to yellow ball
			while not self.see_yellow_ball ():
				self.aicontrol.drive_yaxis (-0.4)
				rospy.sleep (0.5)
		
		if yellow_data.appear:
			## Backward to see three ball
			while not self.see_all_balls ():
				self.aicontrol.drive_xaxis (-0.6)
				rospy.sleep (0.5)
			self.three_ball ()	
		
		self.aicontrol.stop (0.5)

	def two_ball (self):
		red_data = self.detect_bouy (String ('bouy'), String ('r'))
		red_data = red_data.data

		yellow_data = self.detect_bouy (String ('bouy'), String ('y'))
		yellow_data = yellow_data.data

		green_data = self.detect_bouy (String ('bouy'), String ('g'))
		green_data = green_data.data

		if yellow_data.appear:
			## Move yellow to center and backward to see three ball
			print 'YELLOW APPEAR'

			cx = yellow_data.cx[0]
			while not rospy.is_shutdown ():
				yellow_data = self.detect_bouy (String ('bouy'), String ('y'))
				yellow_data = yellow_data.data

				if -0.1 <= cx <= 0.1:
					print 'CENTER'
					print 'BACKWARD TO SEE 3 BALLS'

					self.aicontrol.drive_xaxis (-0.6)
					rospy.sleep (0.5)
				else:
					xImg = yellow_data.cx[0]
					vy = self.aicontrol.adjust (xImg, -0.6, -0.3, 0.3, 0.6)

					self.aicontrol.drive_yaxis (-vy)
					rospy.sleep (0.5)		
		else:
			## Move to see yellow ball nearly
			while not rospy.is_shutdown and not self.see_yellow_ball ():
				print 'YELLOW NOT APPEAR'

				self.aicontrol.drive_xaxis (0.6)
				rospy.sleep (0.5)
		
		self.three_ball ()

	def three_ball (self):
		check = False
		color = 'r'
		while not rospy.is_shutdown ():
			while not self.movement (color):
				print 'HIT BALL FAILED'
			
			print 'HIT BALL SUCCESS'

			if color == 'r':
				color = 'g'
			elif color == 'g':
				color = 'y'
			elif color == 'y':
				break

	def run (self):
		while self.find_num () <= 0:
			self.aicontrol.drive_xaxis (0.6)
			rospy.sleep (1)

		num = self.find_num ()
		print ('Found %d balls', num)

		if num == 3:
			three_ball ()
			
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
					# p.append (self.data.prob[0])
					area.append (self.data.area[0])
					count += 1
				rospy.sleep (0.1)

			if count != 0:
				xImg = self.aicontrol.average (x)
				yImg = self.aicontrol.average (y)
				# prob = self.aicontrol.average (p)
				areaImg = self.aicontrol.average (area)
			else:
				print 'NOT FOUND'

				# self.aicontrol.drive_xaxis (1)
				# rospy.sleep (2)
				# self.aicontrol.stop (1)

				# return False
				continue
			# self.time = self.data.area[0]

			count = 0

			print ('xImg: ', xImg)
			print ('yImg: ', yImg)
			# print ('prob: ', prob)
			print ('area: ', areaImg)
			print ('appear: yes')
			print ('-----------------')
			rospy.sleep (1)

	def test_move (self, color):
		self.aicontrol.fix_zaxis (depth.BOUY_DETECTING)
		count = 0
		while count < 3 and not rospy.is_shutdown ():
			n = self.find_num ()
			print n
			if n == 3:
				count += 1
			elif n == 2:
				print '2'
				## Adjust center of yellow ball
				while not rospy.is_shutdown () and not self.yellow_to_center ():
					print 'YELLOW NOT CENTER'

			self.aicontrol.drive_xaxis (0.7)
			rospy.sleep (0.5)

		print 'FOUND ALL BALLS'
		self.aicontrol.stop (0.5)
		rospy.sleep (1)
		self.startPos = self.aicontrol.get_position ()
		self.startY = self.startPos[1]
		self.startZ = self.startPos[2]
		print self.startPos[1]
		rospy.sleep (5)
		# self.aicontrol.fix_zaxis (-2.6)
		while not self.movement (color) and not rospy.is_shutdown ():
			print 'not pass'
		print 'pass'
		print 'COMPLETE'
		self.aicontrol.stop (1)

if __name__ == '__main__':
	bouy = Bouy ()
	# Bouy.run ()
	# bouy.movement ('y')
	# bouy.get_data ('r')
	bouy.test_move ('g')
	# bouy.test_comeback ()