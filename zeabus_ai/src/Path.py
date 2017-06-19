#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, String
from test_sim_srv_msg.msg import vision_data
from test_sim_srv_msg.srv import path_sim
from AIControl import AIControl

class Path (object):

    def __init__ (self):
        print "Start Mission Path"

        rospy.init_node ('path_node')

        self.aicontrol = AIControl ()
        self.data = None
        self.command = rospy.Publisher ('/cmd_vel', Twist, queue_size=10)
        self.turn_yaw_rel = rospy.Publisher ('/fix/rel/yaw', Float64, queue_size=10)
        
        path_srv = 'vision'
        rospy.wait_for_service (path_srv)
        self.detect_path = rospy.ServiceProxy (path_srv, path_sim)

    def stop (self):
        self.aicontrol.stop (0.1)

    def run (self):
        path = 'path'
        color = 'red'
        vx = 0
        vy = 0
        
        self.stop ()
        self.aicontrol.drive ([1, 0, 0, 0, 0, 0])
        rospy.sleep (11)
        self.stop ()

        while not rospy.is_shutdown ():
            try:
                px = 0
                py = 0
                area = 0
                angle = 0

                for i in range (10):
                    self.data = self.detect_path (String (path), String (color))
                    self.data = self.data.data
                    px = px + self.data.x
                    py = py + self.data.y
                    area = area + self.data.area
                    angle = angle + self.data.angle
                    rospy.sleep (0.01)

                px = px / 10
                py = py / 10
                area =area / 10
                angle = angle / 10

                print ('---------------')
                print ('x: ', px)
                print ('y: ', py)
                print ('area: ', area)
                print ('angle: ', angle)

                if not self.data.isFound:
                    print 'NOT FOUND PATH'

                    self.aicontrol.drive ([0.5, 0, 0, 0, 0, 0])
                    rospy.sleep (1)

                else:
                    print 'FOUND PATH'

                    if self.aicontrol.is_center ([px, py], -0.07, 0.07, -0.3, 0.3):
                        self.stop ()
                        self.aicontrol.turn_yaw_relative (angle)
                        rospy.sleep (0.5)

                        print 'FOUND PATH COMPLETE'
                        break
                    else:
                        vx = self.aicontrol.adjust (px, -0.6, -0.3, 0.3, 0.6)
                        vy = self.aicontrol.adjust (py, -0.6, -0.3, 0.3, 0.6)

                        self.aicontrol.drive ([-vx, -vy, 0, 0, 0, 0])
                    rospy.sleep (0.1)
                    
                self.aicontrol.stop(0.1)

            except rospy.ServiceException as exc:
                print ("Service did not process request: " + str (exc))
                break

if __name__ == '__main__':
    path = Path ()
    path.run ()
    path.stop ()