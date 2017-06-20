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
        path_srv = 'vision2'
        rospy.wait_for_service(path_srv)
        print 'service starts path srv'
        self.detect_path = rospy.ServiceProxy(path_srv, vision_srv)
        #### PATH

        self.aicontrol = AIControl()
        self.hw = Hardware()
        self.oldx = 0
        self.oldy = 0

    def check_point(self, newx, newy):
        if abs(self.oldx - newx) < 0.1 and abs(self.oldy - newy) < 0.1:
            return True
        else:
            return False

    def run (self):

        red_bouy = self.detect_bouy(String('bouy'),String('red'))
        red_bouy = red_bouy.data
        rospy.sleep(2)
        green_bouy = self.detect_bouy(String('bouy'),String('green'))
        green_bouy = green_bouy.data
        rospy.sleep(2)
        bouy_color = ['red', 'green', 'yellow']

        if green_bouy.appear:
            move = 0
            while red_bouy.appear == False and move != 10:
                self.aicontrol.drive_y (0.5)
                red_bouy = self.detect_bouy(String('bouy'),String('red'))
                red_bouy = red_bouy.data
                rospy.sleep(2)
                move += 1
            if red_bouy.appear:
                print 'found red bouy'
            self.aicontrol.stop (2)

        for i in xrange(2):
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

                    if bouy_data.area > 400 : ### near ###
                        print 'near'
                        bc = 0.05
                        sr = 0.2
                    else : ### far ###
                        print 'far'
                        bc = 0.1
                        sr = 0.4

                    if self.aicontrol.is_center([bouy_data.x,bouy_data.y],-bc,bc,-bc,bc) :
                        print bouy_data
                        if bouy_data.area > 600: ### CHANGE ME !!!
                            print 'go to bouy'
                            print 'drive_x 3 meter'
                            self.aicontrol.drive_x (3)
                            rospy.sleep(2)
                            break
                        else:
                            print 'drive_x 0.2 meter so far'
                            self.aicontrol.drive_x (0.2)
                            rospy.sleep(2)
                    else :
                        self.aicontrol.drive([0,vy,vz,0,0,0])
                        print 'set to center'
                        rospy.sleep (sr)

                else :
                    self.aicontrol.stop(0.2)
                    count -= 1
            ### end while ###

            if self.aicontrol.is_fail(count):
                self.aicontrol.drive_x (2)

            move = 0
            self.aicontrol.stop (1)
            print 'stop state after hit bouy'
            print 'backward'

            print 'go to set point'
            if i == 1:
                xx = -2
            else:
                xx = -3
            self.aicontrol.drive_x (xx)
            rospy.sleep(5)

            if i == 0:

                self.aicontrol.drive_z (const.BOUY_DETECTING_DEPTH)
                self.aicontrol.drive_y (-3)
                print 'slide right 3 meter'

                green_bouy = self.detect_bouy(String('bouy'),String('green'))
                green_bouy = green_bouy.data
                rospy.sleep(2)

                while green_bouy.appear == False and move != 20:
                    self.aicontrol.drive_y (-0.1)
                    rospy.sleep(0.5)
                    green_bouy = self.detect_bouy(String('bouy'),String('green'))
                    green_bouy = green_bouy.data
                    rospy.sleep(1)
                    move += 1

                    if not self.check_point(green_bouy.x, green_bouy.y):
                        if self.oldx > green_bouy.x:
                            self.aicontrol.drive ([0,bouy_data.x,bouy_data.y,0,0,0])
                            rospy.sleep (2)
                        else:
                            self.aicontrol.drive ([0,self.oldx,self.oldy,0,0,0])
                            rospy.sleep (2)

                    if self.oldy > green_bouy.y:
                        truey = self.oldy
                        truex = self.oldx
                    self.oldy = green_bouy.y
                    self.oldx = green_bouy.x

            elif i == 1:
                
                self.aicontrol.drive_z (const.BOUY_DETECTING_DEPTH)
                yellow_bouy = self.detect_bouy(String('bouy'),String('yellow'))
                yellow_bouy = yellow_bouy.data
                rospy.sleep(2)

                self.aicontrol.drive_y(1.2)

                while yellow_bouy.appear == False and move != 10:
                    self.aicontrol.stop (2)
                    yellow_bouy = self.detect_bouy(String('bouy'),String('yellow'))
                    yellow_bouy = yellow_bouy.data
                    self.aicontrol.drive_y (0.1)
                    rospy.sleep(1)
                    move += 1
                rospy.sleep(3)

            print 'set point'
            print 'finish ' + bouy_color[i]
            ### end for ###
        print 'finish 2 bouy'
        self.yellow_bouy ()
        # self.find_path()

    def find_path (self):
        self.aicontrol.drive_z (const.PATH_DETECTING_DEPTH)
        path_data = self.detect_path(String('path1'),String('orange'))
        path_data = path_data.data
        print path_data
        count = 80
        found = False
        
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count) :
            if path_data.appear:
                print 'found path'
                self.aicontrol.stop(2)
                break
            else:
                yy = 0.1
                self.aicontrol.drive_x (0.3)
                for i in xrange(8):
                    self.aicontrol.drive_y (yy)
                    path_data = self.detect_path(String('path1'),String('orange'))
                    path_data = path_data.data
                    print path_data
                    if path_data.appear:
                        found = True
                        break
                if not found:
                    self.aicontrol.drive_y(-0.5)
                    yy = -0.1
                    for i in xrange(8):
                        self.aicontrol.drive_y (yy)
                        path_data = self.detect_path(String('path1'),String('orange'))
                        path_data = path_data.data
                        print path_data
                        if path_data.appear:
                            found = True
                            break
                count -= 1

            if found:
                return
            else:
                self.aicontrol.drive_y (0.5)
        return

    def yellow_bouy (self):
        self.aicontrol.drive_z (-3)
        print 'do yellow bouy'
        count = 50
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
                        print 'drive_x 0.3 meter so far'
                        self.aicontrol.drive_x (0.3)
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

        self.find_path()

if __name__ == '__main__':
    bouy_mission = BouyMission()
    #command
    bouy_mission.red_then_green()
    # bouy_mission.yellow_bouy()
    print 'RED AND GREEN BOUY COMPLETE !!'
