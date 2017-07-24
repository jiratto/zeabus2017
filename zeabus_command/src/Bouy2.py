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
		self.count = 0
		self.data = None
		self.fail = 60
		self.nav = None
		self.time = 0.5
		self.startPos = None ## First point that you see three balls
		self.startX = None
		self.startY = None
		self.startZ = None
		self.startYaw = None
		self.destPos = None ## Position that you hit ball
		self.destX = None
		self.destY = None

		bouy_srv = 'vision_bouy'
		nav_srv = 'navigation'
		rospy.wait_for_service (bouy_srv)
		self.detect_bouy = rospy.ServiceProxy (bouy_srv, vision_srv_bouy)
		self.navigation = rospy.ServiceProxy (nav_srv, navigation_srv)

	def find_num (self):
		self.data = self.detect_bouy (String ('bouy'), String ('a'))
		self.data = self.data.data

		return self.data.num

	def yellow_to_center (self):
		print 'ADJUST YELLOW'

		self.data = self.detect_bouy (String ('bouy'), String ('a'))
		self.data = self.data.data

		xImg = self.data.cx

		if xImg == -999:
			self.aicontrol.drive_xaxis (1)
			rospy.sleep (self.time)
			return False

		print ('xImg: ', xImg)

		if -0.04 <= xImg <= 0.04:
			print 'CENTER'

			self.aicontrol.drive_xaxis (1)
			rospy.sleep (self.time)
		else:
			print 'MOVE Y'
			vy = self.aicontrol.adjust (xImg, -0.6, -0.3, 0.3, 0.6)
			
			self.aicontrol.drive_yaxis (vy)
			rospy.sleep (self.time)
			# self.aicontrol.stop (1)

		if self.see_all_balls ():
			self.count += 1

			return True
		else:
			return False

	def see_yellow_ball (self):
		yellow_data = self.detect_bouy (String ('bouy'), String ('o'))
		yellow_data = yellow_data.data

		xImg = yellow_data.cx

		if yellow_data.appear:
			while not rospy.is_shutdown ():
				vy = self.aicontrol.adjust (xImg, -0.6, -0.3, 0.3, 0.6)
				
				self.aicontrol.drive_yaxis (vy)
				rospy.sleep (self.time)
				# self.aicontrol.stop (1)
				yellow_data = self.detect_bouy (String ('bouy'), String ('o'))
				yellow_data = yellow_data.data

				xImg = yellow_data.cx

				print 'MOVE Y'
				print ('xImg: ', xImg)

				if -0.04 <= xImg <= 0.04:
					print 'CENTER'
					break

			return True

		return False

	def see_all_balls (self):
		n = self.find_num ()
		if n < 3:
			return False
		else:
			return True

	def movement (self, color):
		self.data = self.detect_bouy (String ('bouy'), String (color))
		self.data = self.data.data

		xImg = 0
		yImg = 0
		areaImg = 0
		appear = self.data.appear

		if appear:
			xImg = self.data.cx
			yImg = self.data.cy
			areaImg = self.data.area
		else:
			print 'NOT FOUND'

			self.aicontrol.drive_xaxis (1)
			rospy.sleep (2)
			self.fail -= 1

			return False

		print ('xImg: ', xImg)
		print ('yImg: ', yImg)
		print ('area: ', areaImg)
		print ('appear: yes')
		print ('-----------------')

		if self.aicontrol.is_center ([xImg, yImg], -0.04, 0.04, -0.05, 0.05):
			print 'CENTER'

			## check ratio area before hit_and_back ##
			while areaImg < 0.015 and not rospy.is_shutdown () and not self.aicontrol.is_fail (self.fail):
				print 'IN LOOP'

				self.data = self.detect_bouy (String ('bouy'), String (color))
				self.data = self.data.data

				appear = self.data.appear

				if appear:
					xImg = self.data.cx
					yImg = self.data.cy
					areaImg = self.data.area
				else:
					print 'NOT FOUND'

					self.fail -= 1

					continue
				
				print ('xImg: ', xImg)
				print ('yImg: ', yImg)
				print ('area: ', areaImg)

				if areaImg < 0.008:
					vx = (1 / areaImg) / 10
				else:
					print 'AREA more than 0.008 -> ADJUST CENTER'

					if -0.04 <= yImg <= 0.04:
						vx = (1 / areaImg) / 10
					else:
						print 'ADJUST Y-AXIS'

						vy = self.aicontrol.adjust (xImg, -0.6, -0.3, 0.3, 0.6)
						self.aicontrol.drive_yaxis (vy)
						rospy.sleep (self.time)
						self.aicontrol.stop (self.time)
							
						continue

				vx = self.aicontrol.adjust (vx, -0.8, -0.3, 0.3, 0.8)
				self.aicontrol.drive_xaxis (vx)
				rospy.sleep (self.time)

			if self.aicontrol.is_fail (self.fail):
				return True
			
			print 'HIT AND BACK'

			self.aicontrol.stop (1)
			self.hit_and_back ()
			self.comeback_rel ()
			
			return True
		else:
			print 'NOT CENTER'

			temp = self.detect_bouy (String ('bouy'), String (color))

			if -0.05 <= yImg <= 0.05:
				print 'FIX Z'

				vy = self.aicontrol.adjust (xImg, -0.6, -0.3, 0.3, 0.6)
				self.aicontrol.drive_yaxis (vy)
				rospy.sleep (self.time)
				self.aicontrol.stop (0.5)
			else:
				print 'MOVE YZ'
				vy = self.aicontrol.adjust (xImg, -0.6, -0.3, 0.3, 0.6)
				vz = self.aicontrol.adjust (yImg, -1, -0.95, 0.05, 0.1)

				self.aicontrol.drive_zaxis (vz)
				rospy.sleep (self.time)
				
				self.aicontrol.drive_yaxis (vy)
				rospy.sleep (self.time)
			
			return False 

	def hit_and_back (self):
		# self.aicontrol.drive_yaxis (0.4)
		# rospy.sleep (2)

		self.aicontrol.drive_xaxis (0.7)
		rospy.sleep (4)
		self.aicontrol.stop (1)

		print 'HIT BALL!'

		self.aicontrol.drive_xaxis (-1)
		rospy.sleep (6)
		self.aicontrol.stop (1)
		rospy.sleep (1)

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
			self.aicontrol.drive_yaxis (0.2 * dir)
			rospy.sleep (0.5)

		## Go backward
		while not self.see_all_balls ():
			self.aicontrol.drive_xaxis (-1)
			rospy.sleep (0.5)
			self.aicontrol.stop (0.5)

		self.aicontrol.stop (0.5)

		## comeback Z
		print 'COMEBACK Z'
		print self.startZ
		self.aicontrol.fix_zaxis (self.startZ)
		
		print 'COMEBACK COMPLETE'

	def comeback_rel (self):
		print ('start_x: ', self.startX)
		print ('start_y: ', self.startY)
		print ('start_yaw: ', self.startYaw)
		print ('dest_x: ', self.destX)
		print ('dest_y: ', self.destY)

		## comeback XY
		print 'COMEBACK XY'
		# self.aicontrol.drive_x_relative (self.startX, self.startY, self.startYaw)
		self.xy = self.navigation (self.startX, self.startY, self.startYaw)
		self.xy = self.xy.data
		if self.xy:
			print 'COMEBACK COMPLETE'
		else:
			print 'COMEBACK FAILED'

		## comeback Z
		print 'COMEBACK Z'
		print self.startZ
		self.aicontrol.fix_zaxis (self.startZ)

	def set_position_bouy (self):
		self.count = 0

		while self.count < 3 and not rospy.is_shutdown () and not self.aicontrol.is_fail (self.fail):
			n = self.find_num ()
			print n
			if n == 3:
				self.count += 1
				continue
			elif n == 2:
				## Adjust center of yellow ball
				while not rospy.is_shutdown () and not self.yellow_to_center ():
					print 'YELLOW NOT CENTER'

				self.fail -= 1
				continue

			self.aicontrol.drive_xaxis (0.1)
			rospy.sleep (0.5)
			self.fail -= 1

		if self.aicontrol.is_fail (self.fail):
			return

		print 'FOUND ALL BALLS'
		self.aicontrol.stop (0.5)

		## Save start point
		rospy.sleep (1)
		self.startPos = self.aicontrol.get_position ()
		self.startX = self.startPos[0]
		self.startY = self.startPos[1]
		self.startZ = self.startPos[2]
		self.startYaw = self.startPos[5]
		print self.startPos[1]
		rospy.sleep (2)

	def check_not_fail (self):
		if not self.aicontrol.is_fail (self.fail):
			return True
		else:
			return False

	def go_bouy (self):
		# self.aicontrol.fix_zaxis (depth.BOUY_DETECTING)

		## GO TO RED BALL
		self.set_position_bouy ()
		while not self.movement ('r') and not rospy.is_shutdown () and self.check_not_fail ():
			print 'RED BALL'

		## GO TO GREEN BALL
		self.set_position_bouy ()
		while not self.movement ('g') and not rospy.is_shutdown () and self.check_not_fail ():
			print 'GREEN BALL'

		## GO TO YELLOW BALL
		self.set_position_bouy ()
		while not self.movement ('y') and not rospy.is_shutdown () and self.check_not_fail ():
			print 'YELLOW BALL'

		if not self.check_not_fail ():
			return False

		print 'COMPLETE'
		self.aicontrol.stop (1)

		return True

if __name__ == '__main__':
	bouy = Bouy ()
	bouy.run ()