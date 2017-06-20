#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String,Bool
from geometry_msgs.msg import Twist,TwistStamped,Pose,PoseStamped
from zeabus_vision_srv.srv import vision_srv
from zeabus_vision_srv.msg import vision_msg
from AIControl import AIControl
from hardware import Hardware
import depth as const

class BouyMission (object):

    def __init__ (self):
        print "Now do bouy"
        print 'eiei'
        #### BOUY
        ## subscribe vision
        bouy_srv = 'find_obj' ### P'Ink service
        rospy.wait_for_service(bouy_srv)
        print 'service starts bouy srv'
        self.detect_bouy = rospy.ServiceProxy(bouy_srv, vision_srv)
        #### BOUY

        #### PATH
        ## subscribe vision
        bot_srv = 'vision2'
        rospy.wait_for_service(bot_srv)
        print 'service starts bot srv'
        self.detect_path = rospy.ServiceProxy(bot_srv, vision_srv)
        #### PATH

        self.aicontrol = AIControl()
        self.hw = Hardware()
        self.now_pose = [0,0,0,0,0,0]

    def run (self):
        self.aicontrol.drive_z (const.BOUY_DETECTING_DEPTH)

        red_bouy = self.detect_bouy(String('bouy'), String('red'))
        red_bouy = red_bouy.data
        if not red_bouy.appear:
            green_bouy = self.detect_bouy(String('bouy'), String('green'))
            green_bouy = green_bouy.data

            if green_bouy.appear:
                move = 0
                while red_bouy.appear == False and move != 5:
                    self.aicontrol.drive_y (1*mul)
                    red_bouy = self.detect_bouy(String('bouy'),String('red'))
                    red_bouy = red_bouy.data
                    rospy.sleep(2)
                    move += 1
                if red_bouy.appear:
                    print 'found red bouy'
                else:
                    self.aicontrol.drive_y (-5*mul)
                    ##############
                self.aicontrol.stop (2)

        bouy_color = ['red', 'green', 'yellow']

        for i in xrange(1):
            print 'will hit ' + bouy_color[i]
            count = 20

            self.aicontrol.drive_z (const.BOUY_DETECTING_DEPTH) ############# CHANGE ME !!!!
            print 'drive z const complete'

            while not rospy.is_shutdown() and not self.aicontrol.is_fail(count) :
                now_pose = self.aicontrol.get_pose()

                bouy_data = self.detect_bouy(String('bouy'),String(bouy_color[i]))
                bouy_data = bouy_data.data
                print bouy_data

                if bouy_data.appear:

                    vx = (1/bouy_data.area)*500
                    vy = bouy_data.x
                    vz = bouy_data.y

                    if bouy_data.area > 900 : ### near ###
                        print 'near'
                        bc = 0.05
                        sr = 0.2
                    else : ### far ###
                        print 'far'
                        bc = 0.1
                        sr = 0.4

                    if self.aicontrol.is_center([bouy_data.x,bouy_data.y],-bc,bc,-bc,bc) :
                        print bouy_data
                        if bouy_data.area > 1000: ### CHANGE ME !!!
                            print 'go to bouy'
                            print 'drive_x 2 meter'
                            self.aicontrol.drive_x (2)
                            break
                        else:
                            print 'drive_x 0.5 meter so far'
                            self.aicontrol.drive_x (1)
                    else :
                        self.aicontrol.drive([0,vy,vz,0,0,0])
                        print 'set to center'
                        rospy.sleep (sr)

                else :
                    self.aicontrol.drive_x (0.1)
                    count -= 1
            ### end while ###
            self.aicontrol.stop (1)
            print 'stop state after hit bouy'

            if count != 0:
                self.aicontrol.drive_x (-1)
                print 'backward'

                self.aicontrol.drive_y (mul)
                print 'slide to hit green bouy'
                self.aicontrol.turn_yaw_relative (90)

                self.aicontrol.drive_y (-3)
                self.aicontrol.drive_z (const.PATH_DETECTING_DEPTH)
                self.aicontrol.turn_yaw_relative (-90)

            else:
                self.aicontrol.drive_z (const.PATH_DETECTING_DEPTH)


            # print 'backward'

            # print 'go to set point'
            # self.aicontrol.drive_x (-3)
            # rospy.sleep(1)

            # print 'set point'
            # print 'finish ' + bouy_color[i]

            # if i == 0:
            #     move = 0
            #     green_bouy = self.detect_bouy(String('bouy'),String('green'))
            #     green_bouy = green_bouy.data
            #     while green_bouy.appear == False and move != 5:
            #         self.aicontrol.drive_y (1*-mul)
            #         green_bouy = self.detect_bouy(String('bouy'),String('green'))
            #         green_bouy = green_bouy.data
            #         rospy.sleep(2)
            #         move += 1
            #     if green_bouy.appear:
            #         print 'found green bouy'
            #     else:
            #         self.aicontrol.drive_y (-5*-mul)
            #         #####
            # else:
            #     move = 0
            #     yellow_bouy = self.detect_bouy(String('bouy'),String('yellow'))
            #     yellow_bouy = yellow_bouy.data
            #     while yellow_bouy.appear == False and move != 5:
            #         self.aicontrol.drive_y (1*-mul)
            #         yellow_bouy = self.detect_bouy(String('bouy'),String('yellow'))
            #         yellow_bouy = yellow_bouy.data
            #         rospy.sleep(2)
            #         move += 1
            #     if green_bouy.appear:
            #         print 'found yellow bouy'
            #     else:
            #         self.aicontrol.drive_y (-5*-mul)
                    #####
            ### end for ###
        print 'finish 2 bouy'
        self.aicontrol.drive_x(-1)
        self.aicontrol.drive_z(const.PATH_DETECTING_DEPTH)
        self.aicontrol.drive_x (3)
        if self.aicontrol.is_fail(count):
            return False
        else:
            return True
        # self.yellow_bouy()

    def red_then_green (self, q):
        count = 25
        self.aicontrol.drive_z (const.BOUY_DETECTING_DEPTH)

        if q == 'A':
            mul = 1
        else:
            mul = -1

        print 'in red then green'

        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count) :
            print 'count'
            print count
            self.now_pose = self.aicontrol.get_pose()

            bouy_data = self.detect_bouy(String('bouy'),String('red'))
            bouy_data = bouy_data.data
            print bouy_data

            if bouy_data.appear:

                vx = (1/bouy_data.area)*500
                vy = bouy_data.x
                vz = bouy_data.y

                if bouy_data.area > 700 : ### near ###
                    print 'near'
                    bc = 0.05
                    sr = 0.2
                else : ### far ###
                    print 'far'
                    bc = 0.1
                    sr = 0.4

                if self.aicontrol.is_center([bouy_data.x,bouy_data.y],-bc,bc,-bc,bc) :
                    print bouy_data
                    if bouy_data.area > 900: ### CHANGE ME !!!
                        print 'go to bouy'
                        print 'drive_x 2 meter'
                        self.aicontrol.drive_x (2)
                        break
                    else:
                        print 'drive_x 0.5 meter so far'
                        self.aicontrol.drive_x (0.5)
                else :
                    self.aicontrol.drive([0,vy,vz,0,0,0])
                    print 'set to center'
                    rospy.sleep (sr)

            else :
                self.aicontrol.drive_x (0.1)
                count -= 1

        if self.aicontrol.is_fail(count) == False:
            self.aicontrol.go_xyz (self.now_pose[0],self.now_pose[1],self.now_pose[2])
            return False
        else:
            self.aicontrol.drive_x (-1)
            self.aicontrol.drive_y (1.2*mul)
            self.aicontrol.drive_z (-2.3)
            self.aicontrol.drive_x (1)
            print 'FINISH 2 BOUY'

            self.aicontrol.drive_z (const.PATH_DETECTING_DEPTH)
            self.aicontrol.drive_x (1)
            return True

    def do_red (self):
        count = 12
        self.aicontrol.drive_z (const.BOUY_DETECTING_DEPTH)
        print 'in red'

        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count) :
            print 'count'
            print count
            self.now_pose = self.aicontrol.get_pose()

            bouy_data = self.detect_bouy(String('bouy'),String('red'))
            bouy_data = bouy_data.data
            print bouy_data

            if bouy_data.appear:

                vx = (1/bouy_data.area)*500
                vy = bouy_data.x
                vz = bouy_data.y

                if bouy_data.area > 700 : ### near ###
                    print 'near'
                    bc = 0.05
                    sr = 0.2
                else : ### far ###
                    print 'far'
                    bc = 0.1
                    sr = 0.4

                if self.aicontrol.is_center([bouy_data.x,bouy_data.y],-bc,bc,-bc,bc) :
                    print bouy_data
                    if bouy_data.area > 900: ### CHANGE ME !!!
                        print 'go to bouy'
                        print 'drive_x 2 meter'
                        self.aicontrol.drive_x (2)
                        break
                    else:
                        print 'drive_x 0.5 meter so far'
                        self.aicontrol.drive_x (0.5)
                else :
                    self.aicontrol.drive([0,vy,vz,0,0,0])
                    print 'set to center'
                    rospy.sleep (sr)

            else :
                self.aicontrol.drive_y (0.1)
                count -= 1


    def yellow_bouy (self):
        self.aicontrol.drive_z (const.BOUY_DETECTING_DEPTH)
        print 'do yellow bouy'
        count = 20
        self.hw.command ('gripper', 'close')
        print 'open gripper'
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count) :

            bouy_data = self.detect_bouy(String('bouy'),String('yellow'))
            bouy_data = bouy_data.data
            print bouy_data

            if bouy_data.appear:
                vx = (1/bouy_data.area)*500
                vy = bouy_data.x
                vz = bouy_data.y

                if bouy_data.area > 400: ### near ###
                    print 'near'
                    bc = 0.05
                    sr = 0.3
                else : ### far ###
                    print 'far'
                    bc = 0.1
                    sr = 0.5

                if self.aicontrol.is_center([bouy_data.x,bouy_data.y],-bc,bc,-bc,bc) :
                    print bouy_data
                    if bouy_data.area > 600: ### change area value
                        print 'go to bouy'
                        print 'drive_x 2 meter'
                        self.aicontrol.drive_x (2)
                        rospy.sleep (1)
                        self.hw.command ('gripper', 'grab')
                        rospy.sleep (1)
                        print 'grab la na eiei'
                        self.aicontrol.drive_z (const.BOUY_YELLOW_PULL_DEPTH) ####### CHANGE ME !!!
                        print 'drive_z complete'
                        print 'pull down'
                        self.hw.command ('gripper', 'close')
                        self.aicontrol.stop (2)
                        print 'release'
                        self.aicontrol.drive_x (-2)
                        self.aicontrol.drive_z (const.PATH_DETECTING_DEPTH)
                        self.aicontrol.drive_y (-0.8)
                        break
                    else:
                        print 'drive_x 1 meter so far'
                        self.aicontrol.drive_x (1)
                        rospy.sleep(0.5)
                else :
                    self.aicontrol.drive([0,vy,vz,0,0,0])
                    print 'set to center'
                    rospy.sleep (sr)
            else :
                self.aicontrol.stop(0.2)
                count -= 1
        ### end while ###

        self.aicontrol.stop (3)
        print 'stop state after hit bouy'

        self.find_path ()

    def find_path (self):
        count = 6
        self.aicontrol.drive_z (const.PATH_DETECTING_DEPTH)
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count) :
            path_data = self.detect_path(String('path1'),String('orange'))
            path_data = path_data.data
            print path_data
            found = False
            if path_data.appear:
                print 'found path'
                self.aicontrol.stop(2)
                found = True
                break
            else:
                yy = 0.2
                self.aicontrol.drive_x (1)
                for i in xrange(8):
                    self.aicontrol.drive_y (yy)
                    path_data = self.detect_path(String('path1'),String('orange'))
                    path_data = path_data.data
                    print path_data
                    if path_data.appear:
                        found = True
                        return True
                if not found:
                    self.aicontrol.drive_y(-1.2)
                    yy = -0.1
                    for i in xrange(8):
                        self.aicontrol.drive_y (yy)
                        path_data = self.detect_path(String('path1'),String('orange'))
                        path_data = path_data.data
                        print path_data
                        if path_data.appear:
                            found = True
                            return True
                self.aicontrol.drive_x (0.5)
                count -= 1
        return False

if __name__ == '__main__':
    print 'do bouy'
    rospy.init_node('bouy_ai', anonymous=True)
    self.run()
    # self.yellow_bouy()
    print 'RED AND GREEN BOUY COMPLETE !!'
