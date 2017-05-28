#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from zeabus_vision_srv.srv import vision_srv
from zeabus_vision_srv.msg import vision_msg
from AIControl import AIControl
import depth as const

class NavigateMission (object):

    def __init__ (self):
        print "Now do Navigation Channel"
        #### NAVIGATE
        ## subscribe vision
        nav_srv = 'vision1'
        rospy.wait_for_service(nav_srv)
        print 'service starts navigate srv'
        self.detect_nav = rospy.ServiceProxy(nav_srv, vision_srv)

        ### subscribe vision
        bot_srv = 'vision2'
        rospy.wait_for_service(bot_srv)
        print 'service starts nav bot'
        self.bot_nav = rospy.ServiceProxy(bot_srv, vision_srv)

        self.aicontrol = AIControl()

    def run (self, next_is):
        print 'run in navigate'
        count = 50

        gg = next_is

        if next_is:
            self.aicontrol.drive_z (const.NAV_TOP_DETECTING_DEPTH)
            
            while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
                print 'in while'
                nav_data = self.detect_nav(String('navigate'),String('yellow'))
                nav_data = nav_data.data
                print 'DATA NAV TOP CAM'
                print nav_data

                bc = 0.05
                vy = nav_data.x

                if nav_data.appear :
                    print 'found'
                    vy = nav_data.x
                    vz = nav_data.y
                    print nav_data

                    if self.aicontrol.is_center([nav_data.x,0],-bc,bc,-bc,bc) :
                        print 'center'
                        self.aicontrol.drive_x (0.1)

                        bot_data = self.bot_nav(String('navigate'),String('yellow'))
                        bot_data = bot_data.data
                        print 'DATA NAV BOT CAM'
                        print bot_data

                        self.aicontrol.drive_z (const.NAV_BOT_DETECTING_DEPTH)
                        c = 50
                        
                        while not bot_data.appear and not self.aicontrol.is_fail(c):
                            self.aicontrol.drive_x (0.1)
                            rospy.sleep(1)
                            bot_data = self.bot_nav(String('navigate'),String('yellow'))
                            bot_data = bot_data.data
                            print 'DATA NAV BOT CAM'
                            print bot_data
                            c -= 1

                        print 'will stop'
                        self.aicontrol.stop(1)
                        self.aicontrol.drive_z (const.NAV_ROLL_DEPTH) #### CHANGE ME !! ####
                        self.aicontrol.stop(1)
                        print 'stop wait to roll'
                        gg = False
                        break

                    else :
                        print 'not center'
                        self.aicontrol.drive ([0,vy,0,0,0,0])
                        rospy.sleep (1)
                ### end while ###
                else :
                    print 'not found'
                    self.aicontrol.drive_x (0.1)
                    rospy.sleep(1)
                    count -= 1
            
            rospy.sleep (0.25)
        
        print 'will roll'
        self.aicontrol.stop(1)
        self.aicontrol.drive_z (const.NAV_ROLL_DEPTH) ###### CHANGE ME !!!
        print 'drive z'
        print 'drive style'
        ### style ###
        self.aicontrol.roll (3)
        self.aicontrol.stop (3)
        print 'finish navigation channel'
        return

if __name__ == '__main__':
    navigate_mission = NavigateMission()
    navigate_mission.run()
    print "finish Navigation Channel"
