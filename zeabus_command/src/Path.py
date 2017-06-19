#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, String
from zeabus_vision_srv_msg.msg import vision_msg_default
from zeabus_vision_srv_msg.srv import vision_srv_default
from AIControl import AIControl

class Path (object):

    def __init__ (self):
        print "Start Mission Path"

        rospy.init_node ('path_node')

        self.aicontrol = AIControl ()
        self.data = None
        self.command = rospy.Publisher ('/zeabus/cmd_vel', Twist, queue_size=10)
        self.turn_yaw_rel = rospy.Publisher ('/fix/rel/yaw', Float64, queue_size=10) 	
        
        path_srv = 'vision'
        rospy.wait_for_service (path_srv)
        self.detect_path = rospy.ServiceProxy (path_srv, vision_srv_default)

    def stop (self):
        self.aicontrol.stop (2)

    def run (self):
        path = 'path'
        color = 'red'
        vx = 0
        vy = 0
        
        self.stop ()
        # self.aicontrol.drive ([1, 0, 0, 0, 0, 0])
        # rospy.sleep (11)
        # self.stop (2)

        while not rospy.is_shutdown ():
            try:
                px = 0
                py = 0
                area = 0
                angle = 0
                count = 0

                for i in range (10):
                    self.data = self.detect_path (String (path), String (color))
                    self.data = self.data.data
                    if not px == -999:
		                px = px + self.data.x
		                py = py + self.data.y
		                area = area + self.data.area
		                angle = angle + self.data.angle
		                count = count + 1
		                rospy.sleep (0.01)

                if not count == 0:
		        	px = px / count
		        	py = py / count
		        	area = area / count
		        	angle = angle / count
                else:
	            	px = -999
	            	py = -999
	            	area = -999
	            	angle = -999
 
                print ('---------------')
                print ('x: ', px)
                print ('y: ', py)
                print ('area: ', area)
                print ('angle: ', angle)

                if not self.data.appear:
                    print 'NOT FOUND PATH'

                    self.aicontrol.drive ([0.5, 0, 0, 0, 0, 0])
                    rospy.sleep (4)

                else:
                    print 'FOUND PATH'

                    if self.aicontrol.is_center ([px, py], -0.2, 0.2, -0.3, 0.3):
                        self.stop ()
                        self.aicontrol.turn_yaw_relative (angle)

                        print 'FOUND PATH COMPLETE'
                        break
                    else:
                        vx = self.aicontrol.adjust (px, -0.5, -0.2, 0.2, 0.5)
                        vy = self.aicontrol.adjust (py, -0.5, -0.2, 0.2, 0.5)

                        self.aicontrol.drive ([-vx, -vy, 0, 0, 0, 0])
                    rospy.sleep (2)
                    
                self.stop ()

            except rospy.ServiceException as exc:
                print ("Service did not process request: " + str (exc))
                break

if __name__ == '__main__':
    path = Path ()
    path.run ()
    path.stop ()
    print 'end'