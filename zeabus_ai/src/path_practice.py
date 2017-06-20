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
        print "Now do gate"
        #### PATH
        ## subscribe vision
        bot_srv = 'vision2'
        rospy.wait_for_service(bot_srv)
        print 'service starts bot srv'
        self.detect_path = rospy.ServiceProxy(bot_srv, vision_srv)
        #### PATH

        #### NAV
        ### subscribe vision
        nav_srv = 'vision1'
        rospy.wait_for_service(nav_srv)
        print 'service starts nav bot'
        self.top_nav = rospy.ServiceProxy(nav_srv, vision_srv)
        #### NAV

        #### BOT NAV
        ### subscribe vision
        bot_srv = 'vision2'
        rospy.wait_for_service(bot_srv)
        print 'service starts nav bot'
        self.bot_nav = rospy.ServiceProxy(bot_srv, vision_srv)
        #### BOT NAV

        #### BOUY
        ## subscribe vision
        bouy_srv = 'find_obj'
        rospy.wait_for_service(bouy_srv)
        print 'servive starts bouy srv'
        self.detect_bouy = rospy.ServiceProxy(bouy_srv, vision_srv)
        #### BOUY

        self.aicontrol = AIControl()

    def run (self):

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

                if self.aicontrol.is_center([path_data.x,path_data.y],-0.06,0.06,-0.06,0.06) and -3 < binn_data.angle[i] < 3:
                    self.angle = path_data.angle
                    print self.angle
                    self.aicontrol.turn_yaw_relative(self.angle)
                    print 'Center'
                    break
                else :
                    print 'Not Center'
                    vx = self.aicontrol.adjust (path_data.x, -0.3, -0.1, 0.1, 0.3)
                    vy = self.aicontrol.adjust (path_data.y, -0.3, -0.1, 0.1, 0.3)

                    self.aicontrol.drive ([vx,vy,0,0,0,0])
                    rospy.sleep (1)
            else :
                print 'not found'
                # self.aicontrol.stop(0.25)
                count -= 1

        if self.aicontrol.is_fail(count) :
            print 'Find Path Fail'
            return False

        print 'Find Path Complete'
        return True

    def find_bouy (self):
        count = 16
        self.aicontrol.drive_z (const.BOUY_DETECTING_DEPTH)
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):

            print count

            red_bouy = self.detect_bouy(String('bouy'),String('red'))
            red_bouy = red_bouy.data
            rospy.sleep(1)
            print 'RED BOUY DATA'
            print red_bouy

            # green_bouy = self.detect_bouy(String('bouy'),String('green'))
            # green_bouy = green_bouy.data
            # rospy.sleep(1)
            # print 'GREEN BOUY DATA'
            # print green_bouy

            # yellow_bouy = self.detect_bouy(String('bouy'),String('yellow'))
            # yellow_bouy = yellow_bouy.data
            # rospy.sleep(1)
            # print 'YELLOW BOUY DATA'
            # print yellow_bouy

            # if red_bouy.appear or green_bouy.appear or yellow_bouy.appear:
            if red_bouy.appear:
                print 'FOUND SOME BOUY'
                found = True
                break
            else:
                print 'NOT FOUND BOUY'
                self.aicontrol.drive_x (0.5)
                count -= 1

        if self.aicontrol.is_fail (count):
            found = False

        return found

    def find_nav (self):
        self.aicontrol.drive_z (const.NAV_TOP_DETECTING_DEPTH) #### CHANGE ME !!!
        count = 40

        saw = False
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            bot_data = self.bot_nav(String('navigate'),String('yellow'))
            bot_data = bot_data.data
            print 'NAVIGATE DATA BOT CAM'
            print bot_data

            nav_data = self.top_nav(String('navigate'),String('yellow'))
            nav_data = nav_data.data
            print 'NAVIGATE DATA TOP CAM'
            print nav_data

            if nav_data.appear:
                print 'FOUND NAV BY TOP CAM'
                return True
            else:
                count -= 1

            if bot_data.appear:
                print 'FOUND NAV BY BOT CAM'
                return False

            self.aicontrol.drive_x (0.3)
            rospy.sleep (1)

        # self.aicontrol.stop(2)
        return True


if __name__ == '__main__':
    print 'start path'
    # rospy.init_node('path_ai', anonymous=True)
    self.run ()
    print "finish path"
