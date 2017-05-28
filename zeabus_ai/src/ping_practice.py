#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from zeabus_hydrophone.srv import hydro_info
from zeabus_hydrophone.msg import hydro_msg
from AIControl import AIControl
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

    def reset(self):
        reset_hy = rospy.Publisher('/hydro_status', Bool, queue_size = 1)
        while reset_hy.get_num_connections() == 0:
            rospy.sleep (1)
        reset_hy.publish (True)

    def listening(self, data): #### call back
        self.hy = data

    def convert(self, azi):
        print 'covert degree to turn'
        azi = -azi
        real = azi - 45
        if real < -180:
            real = real + 360
        return real

    def ping_check(self):
        print "listen hydrophone"
        self.aicontrol.stop (5)
        self.reset()

        real_degree = self.convert(self.hy.azi)
        print self.hy.azi
        print real_degree

        dis = 3
        self.aicontrol.turn_yaw_relative (real_degree)

        goal = False

        while goal != True:
            print 'listen pinger'
            state = self.aicontrol.get_pose()
            my_yaw = state[5]
            print '*****************'

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
                    if dis == 0:
                        dis = 0.5
                    rospy.sleep(1)
                else:
                    self.aicontrol.turn_yaw_relative (real_degree)
                    print 'turn'
                print'***********************'
            else:
                print 'yung mai tung'

            print self.hy
            rospy.sleep (5)

            if self.hy.elv < 40:
                dis = 0.2
            if self.hy.elv < 25 and self.hy.stop:
                print self.hy.elv
                self.aicontrol.drive_z (-0.05)
                goal = True
        return

if __name__=='__main__':
    print 'hydrophone'
    rospy.init_node('ping_ai')
    self.run()
