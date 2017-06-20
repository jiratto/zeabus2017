#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from zeabus_vision_srv.srv import vision_srv
from zeabus_vision_srv.msg import vision_msg
from AIControl import AIControl
import depth as const
import direction as tis

class GateMission (object):

    def __init__ (self):
        print "Now do gate"
        #### PATH
        ## subscribe vision
        bot_srv = 'vision2'
        rospy.wait_for_service(bot_srv)
        print 'service starts bot srv'
        self.detect_path = rospy.ServiceProxy(bot_srv, vision_srv)
        #### PATH

        self.aicontrol = AIControl()

    def run (self):
        print 'drive z'
        self.aicontrol.drive_z (const.GATE_PASS_DEPTH)

        self.aicontrol.turn_yaw_absolute (tis.GATE_DIRECTION)
        rospy.sleep (2)
        print 'tuer yaw absolute complete'
        self.aicontrol.drive_x (8)

        found = False
        count = 15
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):

            print 'PATH DATA'
            path_data = self.detect_path(String('path1'), String('orange'))
            path_data = path_data.data
            print path_data
            print '--------'

            if path_data.appear :
                print 'found path'
                self.aicontrol.stop(2)
                found = True
                break
            else:
                print 'not found'
                self.aicontrol.drive_x (0.3)
                count -= 1

        # if self.aicontrol.is_fail(count):
        #     found = False
            # self.aicontrol.drive_x (1)
            # self.aicontrol.drive_y (1)
            # path_data = self.detect_path(String('path1'),String('orange'))
            # path_data = path_data.data
            # print path_data
            # if path_data.appear:
            #     found = True
            # if not found:
            #     self.aicontrol.drive_y(-2)
            #     path_data = self.detect_path(String('path1'),String('orange'))
            #     path_data = path_data.data
            #     print path_data
            #     if path_data.appear:
            #         found = True

        return found


if __name__ == '__main__':
    print 'start gate'
    gate_mission = GateMission()
    #command
    gate_mission.run_without_vision()
    print "finish gate"
