#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, String
from zeabus_vision_srv_msg.msg import vision_msg_default
from zeabus_vision_srv_msg.srv import vision_srv_default
from AIControl import AIControl
import Depth as depth

class Path (object):

    def __init__ (self):
        print "Start Mission Path"

        # rospy.init_node ('path_node')

        self.aicontrol = AIControl ()
        self.data = None  
        self.isFail = 30
        
        path_srv = 'vision'
        rospy.wait_for_service (path_srv)
        self.detect_path = rospy.ServiceProxy (path_srv, vision_srv_default)

    def stop (self):
        self.aicontrol.stop (2)

    def find (self):
        path = 'path'
        color = 'red'
        vx = 0
        vy = 0
        check = 0
        self.aicontrol.fix_zaxis (depth.PATH_DETECTING)
        rospy.sleep (3)
        # self.aicontrol.drive_xaxis (1)
        # rospy.sleep (11)
        # self.stop (2)

        while not rospy.is_shutdown () and not self.aicontrol.is_fail (self.isFail):
            self.stop ()

            print "IN LOOP"

            px = []
            py = []
            area = []
            angle = []
            count = 0

            if check == 2:
                break

            for i in range (10):
                print "IN AVG"
                self.data = self.detect_path (String (path), String (color))
                print "GET DATA"
                self.data = self.data.data
                if not px == -999:
                      px.append (self.data.x)
                      py.append (self.data.y)
                      area.append (self.data.area)
                      angle.append (self.data.angle)
                      count = count + 1
                      rospy.sleep (0.01)

            if count == 0:
                px = -999
                py = -999
                area = -999
                angle = -999
 
            avrPx = self.aicontrol.average (px)
            avrPy = self.aicontrol.average (py)
            avrArea = self.aicontrol.average (area)
            avrAngle = self.aicontrol.average (angle)

            print ('---------------')
            print ('x: ', avrPx)
            print ('y: ', avrPy)
            print ('area: ', avrArea)
            print ('angle: ', avrAngle)

            if not self.data.appear:
                print 'NOT FOUND PATH'

                self.aicontrol.drive_xaxis (0.7)
                rospy.sleep (1)
                self.isFail -= 1
            else:
                print 'FOUND PATH'

                if self.aicontrol.is_center ([avrPx, avrPy], -0.2, 0.2, -0.1, 0.1):
                    self.stop ()
                    self.aicontrol.turn_yaw_relative (avrAngle)
                    check += 1
                else:
                    vx = self.aicontrol.adjust (avrPx, -0.5, -0.1, 0.1, 0.5)
                    vy = self.aicontrol.adjust (avrPy, -0.5, -0.1, 0.1, 0.5)

                self.aicontrol.drive ([vx, vy, 0, 0, 0, 0])
            rospy.sleep (4)

        print (self.check_path ())

    def check_path (self):
        if not self.aicontrol.is_fail (self.isFail):
            return True
        else:
            return False

            
# if __name__ == '__main__':
#     path = Path ()
#     print "EIEI"
#     path.run ()
#     path.stop ()
#     print 'end'