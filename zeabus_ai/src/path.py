#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from zeabus_vision_srv.srv import vision_srv
from zeabus_vision_srv.msg import vision_msg
from AIControl import AIControl
import depth as const

class PathMission (object):

    def __init__ (self):
        print "Mission : Path"
        #### PATH
        ## subscribe vision
        path_srv = 'vision2'
        rospy.wait_for_service(path_srv)
        print 'service starts path srv'
        self.detect_path = rospy.ServiceProxy(path_srv, vision_srv)
        #### PATH

        #### BOUY
        ## subscribe vision
        # bouy_srv = 'find_obj'
        # rospy.wait_for_service(bouy_srv)
        # print 'servive starts bouy srv'
        # self.detect_bouy = rospy.ServiceProxy(bouy_srv, vision_srv)
        #### BOUY

        #### NAV
        ## subscribe vision
        # nav_srv = 'vision1'
        # rospy.wait_for_service(nav_srv)
        # print 'servive starts nav srv'
        # self.detect_nav = rospy.ServiceProxy(nav_srv, vision_srv)
        #### NAV

        #### BOT NAV
        ### subscribe vision
        # bot_srv = 'vision2'
        # rospy.wait_for_service(bot_srv)
        # print 'service starts nav bot'
        # self.bot_nav = rospy.ServiceProxy(bot_srv, vision_srv)
        #### BOT NAV

        self.aicontrol = AIControl()
        self.angle=0

    def goto_path (self):

        self.aicontrol.drive_z(const.PATH_DETECTING_DEPTH)
        print 'Go to Path'
        count = 30

        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            path_data = self.detect_path(String('path1'),String('orange'))
            path_data = path_data.data
            print 'PATH DATA'
            print path_data
            rospy.sleep (1)

            if path_data.appear :
                print 'found'

                if self.aicontrol.is_center([path_data.x,path_data.y],-0.06,0.06,-0.06,0.06):
                    self.angle = path_data.angle
                    print self.angle
                    print 'Center'
                    break
                else :
                    print 'Not Center'
                    vx = self.aicontrol.adjust (path_data.x, -0.3, -0.1, 0.1, 0.3)
                    vy = self.aicontrol.adjust (path_data.y, -0.3, -0.1, 0.1, 0.3)

                    self.aicontrol.drive ([vx,vy,0,0,0,0])
                    rospy.sleep (0.5)
            else :
                print 'not found'
                self.aicontrol.stop(0.25)
                count -= 1

        if self.aicontrol.is_fail(count) :
            print 'Find Path Fail'
            return False

        print 'Find Path Complete'
        return True

    def run(self, obj):
        print 'Start Path Mission'

        if(self.goto_path()):
            print 'turn_yaw'
            print self.angle
            self.aicontrol.stop(2)
            self.aicontrol.turn_yaw_relative(self.angle)
            rospy.sleep(6)
        else :
            self.aicontrol.drive_x (2)

        # if obj == 'bouy':
        #     count = 30
        #     self.aicontrol.drive_z (const.BOUY_DETECTING_DEPTH)
        #     while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):

        #         red_bouy = self.detect_bouy(String('bouy'),String('red'))
        #         red_bouy = red_bouy.data
        #         rospy.sleep(1)
        #         green_bouy = self.detect_bouy(String('bouy'),String('green'))
        #         green_bouy = green_bouy.data
        #         rospy.sleep(1)
        #         # yellow_bouy = self.detect_bouy(String('bouy'),String('yellow'))
        #         # yellow_bouy = yellow_bouy.data
        #         # rospy.sleep(1)

        #         print 'RED BOUY DATA'
        #         print red_bouy
        #         print 'GREEN BOUY DATA'
        #         print green_bouy
        #         # print 'YELLOW BOUY DATA'
        #         # print yellow_bouy

        #         if red_bouy.appear:
        #             print 'RED FOUND BOUY'
        #             return
        #             break
        #         elif green_bouy.appear:
        #             print 'GREEN FOUND BOUY'
        #             for i in xrange(10):
        #                 red_bouy = self.detect_bouy(String('bouy'),String('red'))
        #                 red_bouy = red_bouy.data
        #                 rospy.sleep(2)
        #                 if red_bouy.appear:
        #                     print 'RED FOUND BOUY'
        #                     break
        #                 else:
        #                     self.aicontrol.drive ([0,0.5,0,0,0,0])
        #                     rospy.sleep (1)
        #             return
        #             break
        #         else:
        #             self.aicontrol.drive_x (0.1)
        #             print 'forward'
        #             count -= 1

        # if obj == 'navigate':
        #     self.aicontrol.drive_x (2)
        #     self.aicontrol.drive_z (const.NAV_TOP_DETECTING_DEPTH) #### CHANGE ME !!!
        #     count = 30

        #     saw = False
        #     while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
        #         bot_data = self.bot_nav(String('navigate'),String('yellow'))
        #         bot_data = bot_data.data
        #         print 'NAVIGATE DATA BOT CAM'
        #         print bot_data

        #         nav_data = self.nav_srv(String('navigate'),String('yellow'))
        #         nav_data = nav_data.data
        #         print 'NAVIGATE DATA TOP CAM'
        #         print nav_data

        #         if nav_data.appear:
        #             print 'FOUND NAV BY TOP CAM'
        #             return True
        #         else:
        #             count -= 1

        #         if bot_nav.appear:
        #             print 'FOUND NAV BY BOT CAM'
        #             self.aicontrol.drive_x (-0.3)
        #             return False

        #         self.aicontrol.drive_x (0.2)
        #         rospy.sleep (1)

        # self.aicontrol.stop(2)
        # return


if __name__ == '__main__':
    path_mission = PathMission()
    #command
    path_mission.run()
