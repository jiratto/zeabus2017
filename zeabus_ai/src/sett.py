#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from zeabus_vision_setcourse.srv import SetCourse_Srv
from zeabus_vision_setcourse.msg import SetCourse_Msg
from AIControl import AIControl
from hardware import Hardware
import depth as const

class SettMission (object):

    def __init__ (self):
        # rospy.init_node('sett_srv', anonymous=True)
        print "Now do Set Course"
        #### SETT
        ## subscribe vision
        sett_srv = 'setcourse_srv'
        rospy.wait_for_service(sett_srv)
        print 'service starts top srv'
        self.detect_sett = rospy.ServiceProxy(sett_srv, SetCourse_Srv)
        #### SETT

        self.aicontrol = AIControl()
        self.hw = Hardware()

    def find_sett (self):
        count = 50

        self.aicontrol.drive_z (const.SET_DETECTING_DEPTH) ##### CHANGE ME !!

        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            sett_data = self.detect_sett(String('setcourse'),String('big'))
            sett_data = sett_data.data
            print sett_data

            if len(sett_data.appear) != 0 and sett_data.appear[0]:
                print 'found'
                if sett_data.area[0] > 15000: ###### CHANGE ME !!
                    print 'near'
                    bc = 0.1
                else:
                    print 'far'
                    bc = 0.3

                if self.aicontrol.is_center([sett_data.x[0],sett_data.y[0]],-bc,bc,-bc,bc):
                    if sett_data.area[0] > 16000: ###### CHANGE ME !!
                        print 'should see 4 sq, find big complete !!'
                        break
                    else:
                        print 'forward'
                        self.aicontrol.drive_x (0.05)
                else:
                    self.aicontrol.drive ([0,sett_data.x[0],sett_data.y[0],0,0,0])
                    rospy.sleep(0.5)
                    print 'set to center'
                    self.aicontrol.stop (1)
            else:
                self.aicontrol.stop (1)
                print 'not found'
                count -= 1

    def find_sq (self, yd, zd, la): ### y_direction & z_direction & launcher
        count = 20
        saw = False

        # self.aicontrol.drive_y (0.6*yd/0.05)

        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            sq_data = self.detect_sett(String('setcourse'),String('small'))
            sq_data = sq_data.data
            print sq_data

            if len(sq_data.appear) == 1:

                if self.aicontrol.is_center ([sq_data.x[0],sq_data.y[0]],-0.05,0.05,-0.05,0.05):
                    if sq_data.area[0] > 6000:
                        print 'forward to fire'
                        self.hw.command (String(la), String('fire'))
                        self.hw.command (String(la), String('close'))
                        print 'fire complete'
                        self.aicontrol.stop (10)
                        self.aicontrol.drive_x (-2)
                        break
                    else:
                        print 'forward'
                        self.aicontrol.drive_x (0.02)
                else:
                    print 'move'
                    self.aicontrol.drive ([0,sq_data.x[0],sq_data.y[0],0,0,0])
                    rospy.sleep(0.5)
            elif len(sq_data.appear) == 2 or len(sq_data.appear) == 3:
                self.aicontrol.drive_x (0.2)
            else:
                state = self.aicontrol.get_pose()
                if -4 < state[2] < -2: ###### CHANGE ME !!!
                    self.aicontrol.drive ([0,0,zd,0,0,0])
                    rospy.sleep(0.5)
                self.aicontrol.drive ([0.2,yd,0,0,0,0])
                rospy.sleep (0.2)
        return

    def run (self):
        v = 0.05
        eq = ['fire_left', 'fire_right']
        self.find_sett ()
        self.find_sq (-v, v, eq[0])  #### right top - +
        self.find_sett ()
        self.find_sq (v, -v, eq[1])
        # self.find_sq (v, v, eq[1])   #### left top + +
        # self.find_sq (v, -v, eq[1])  #### left bot + -
        # self.find_sq (-v, -v, eq[0]) #### right bot - -

if __name__ == '__main__':
    sett_mission = SettMission()
    #command
    # sett_mission.run(['N','W'])
    print "finish sett"
