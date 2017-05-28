#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from zeabus_hydrophone.srv import hydro_info
from zeabus_hydrophone.msg import hydro_msg
from zeabus_vision_bin.srv import Bin_Srv
from zeabus_vision_bin.msg import Bin_Msg
from zeabus_vision_setcourse.srv import SetCourse_Srv
from zeabus_vision_setcourse.msg import SetCourse_Msg
from AIControl import AIControl
from binn_practice import BinnMission
from sett_practice import SettMission
from hardware import Hardware
import depth as const
import math

class PingerMission(object):
    def __init__(self):
        print 'pinger init'
        self.aicontrol = AIControl()
        self.hy = hydro_msg()
        self.check = Bool
        ### subscribe hydrophone ###
        rospy.Subscriber ('/hydro', hydro_msg, self.listening)

        #### BINN
        ## subscribe vision
        srv_name = 'bin_srv'
        rospy.wait_for_service(srv_name)
        print 'service starts binn'
        self.detect_binn = rospy.ServiceProxy(srv_name, Bin_Srv)

        #### SETT
        ## subscribe vision
        sett_srv = 'setcourse_srv'
        rospy.wait_for_service(sett_srv)
        print 'service starts top srv'
        self.detect_sett = rospy.ServiceProxy(sett_srv, SetCourse_Srv)

        self.got_data = False

    def reset(self):
        reset_hy = rospy.Publisher('/hydro_status', Bool, queue_size = 1)
        while reset_hy.get_num_connections() == 0:
            rospy.sleep (1)
        reset_hy.publish (True)

    def listening(self, data): #### call back
        self.got_data = True
        self.hy = data

    def convert(self, azi):
        print 'covert degree to turn'
        azi = -azi
        real = azi - 45
        if real < -180:
            real = real + 360
        return real

    def check_data(self):
        return self.got_data

    def ping_check(self):
        self.aicontrol.drive_z (const.PING_DETECTING_DEPTH)
        print "listen hydrophone"
        self.aicontrol.stop (3)
        self.reset()
        self.aicontrol.stop (3)

        real_degree = self.convert(self.hy.azi)
        print self.hy.azi
        print real_degree

        dis = 7
        self.aicontrol.turn_yaw_relative (real_degree)

        goal = False
        count = 50

        while goal != True and not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            # if not self.got_data:
            #     self.aicontrol.drive_x (0.2)
            print 'listen pinger'
            state = self.aicontrol.get_pose()
            my_yaw = state[5]
            print '*****************'
            print count
            if self.hy.distance != -999:

                if self.aicontrol.stop_turn():
                    real_degree = self.convert(self.hy.azi)
                    print self.hy.azi
                    print 'real_degree'
                    print real_degree
                    print 'stop status'
                    print self.aicontrol.stop_turn()
                    if real_degree > -10 and real_degree < 10:
                        self.aicontrol.drive_x (dis)  ### control distance by azi || elv
                        print 'drive'
                        dis -= 0.5
                        if dis <= 0:
                            dis = 0.5
                        rospy.sleep(1)
                    else:
                        # if real_degree > 30:
                        #     real_degree = 30
                        # elif real_degree < -30:
                        #     real_degree = -30
                        self.aicontrol.turn_yaw_relative (real_degree)
                        print 'turn'
                    print'***********************'
                else:
                    print 'yung mai tung'

                print self.hy
                rospy.sleep (5)

                if self.hy.elv < 40:
                    dis = 0.5
                if self.hy.stop:
                    print self.hy.elv
                    goal = True
                ##### add stop state
            count -= 1
        return goal

    def find_binn(self):
        count = 10
        self.aicontrol.drive_z (const.BIN_DETECTING_DEPTH)
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            binn_data = self.detect_binn(String('bin'),String('white'))
            binn_data = binn_data.data
            print binn_data

            if len(binn_data.appear) == 1 or len(binn_data.appear) == 2:
                print 'found binn'
                return True
            else:
                print 'not found'
                count -= 1
                # self.aicontrol.drive_x (0.04)
        return False

    def find_sett(self):
        self.aicontrol.drive_z (const.SET_DETECTING_DEPTH) #### CHANGE ME !!!
        count = 12
        print 'set course'
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            print count
            found = 0
            for i in xrange(5):
                sett_data = self.detect_sett(String('setcourse'),String('small'))
                sett_data = sett_data.data
                print sett_data
                if len(sett_data.appear) != 0:
                    found += 1

            if found >= 3:
                print 'FOUND SETT !!'
                return True
            else:
                print 'not found'
                self.aicontrol.turn_yaw_relative(-30)
                rospy.sleep(1)
                self.aicontrol.stop(0.1)
                count -= 1

        return False

    def con_find(self):
        hw = Hardware()

        is_at_bin = False
        if self.find_binn():
            is_at_bin = True
            self.do_binn()

        if self.find_sett():
            is_at_bin = True
            self.do_sett()

        elif is_at_bin:
            is_at_bin = True
            hw.command('drop_right', 'fire')
            hw.command('drop_left', 'fire')
            hw.command('fire_right', 'fire')
            hw.command('fire_left', 'fire')

        return is_at_bin

    def run(self):
        self.aicontrol.stop(2)
        self.aicontrol.drive_z (const.PING_DETECTING_DEPTH)
        print 'stop to listen pinger'
        self.ping_check()
        print 'above pinger'

        found_binn = self.find_binn()
        if found_binn:
            self.do_binn()
            self.aicontrol.drive_z (const.SET_DETECTING_DEPTH)
            self.find_sett()
            self.do_sett()
            self.aicontrol.drive_z (const.PING_DETECTING_DEPTH)
            rospy.sleep(30)
            self.reset()
            self.ping_check()
            self.aicontrol.drive_z (const.PING_FLOATING_DEPTH)
        else:
            self.aicontrol.drive_z (const.PING_FLOATING_DEPTH)
            rospy.sleep(30)
            self.aicontrol.drive_z (const.PING_DETECTING_DEPTH)
            self.reset()
            self.ping_check()
            self.con_find()

        self.aicontrol.drive_z (const.FINISH)

    def do_binn(self):
        binn_mission = BinnMission()
        binn_mission.run(0)

    def do_sett(self):
        sett_mission = SettMission()
        sett_mission.run()

if __name__=='__main__':
    print 'hydrophone'
    # rospy.init_node('ping_ai')
    ping=PingerMission()
    ping.run()
