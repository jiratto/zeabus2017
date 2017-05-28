#!/usr/bin/env python

import rospy
import math
from gate import GateMission
from navigate import NavigateMission
from pingping import PingerMission
from AIControl import AIControl
from modbus_ascii_ros.msg import Switch
#### practice
from path_practice import PathMission
from bouy_practice import BouyMission
from binn_practice import BinnMission
from sett_practice import SettMission
#### practice
import depth as const
import direction as tis

def start (switch):
    global check
    global ever_turn_on
    if switch.motor_switch == False:
        check = False
        if ever_turn_on:
            rospy.signal_shutdown('Turn off switch')
    else:
        check = True
        ever_turn_on = True

if __name__ == '__main__':
    rospy.sleep (2)
    global check
    global ever_turn_on
    ever_turn_on = False
    rospy.Subscriber("/switch/data", Switch, start, queue_size = 1)
    rospy.init_node('main_ai')
    print "init node complete"

    print 'DO AI EIEI YEAH !!'
    #### inherit phase
    aicontrol = AIControl()
    gate_mission = GateMission()
    print '1'
    path_prac_mission = PathMission()
    print '2'
    bouy_prac_mission = BouyMission()
    print '3'
    navigate_mission = NavigateMission()
    print '4'
    binn_prac_mission = BinnMission()
    print '6'
    sett_prac_mission = SettMission()
    print '7'
    print 'inherit complete'

    while not check and not rospy.is_shutdown():
        rospy.sleep(0.1)

    path_complete = False
    path_one_found = gate_mission.run ()
    if path_one_found:
        print 'CAN FOLLOW PATH'
        path_complete = path_prac_mission.run()
    
    if not path_complete:
        print 'TURN TO FIND BOUY'
        aicontrol.turn_yaw_absolute (tis.BOUY_DIRECTION)

    red_status = path_prac_mission.find_bouy()
    if red_status:
        print 'DO RED BOUY'
        bouy_prac_mission.do_red()
    else:
        aicontrol.drive_z (const.PATH_DETECTING_DEPTH)
        aicontrol.turn_yaw_absolute (tis.NAV_DIRECTION)
    aicontrol.drive_y (-1.2)
    
    path_complete = False
    path_two_found = bouy_prac_mission.find_path ()
    if path_two_found:
        print 'CAN FOLLOW PATH'
        path_complete = path_prac_mission.run()
    
    if not path_complete:
        print 'TURN TO FIND PATH'
        aicontrol.turn_yaw_absolute (tis.NAV_DIRECTION)
    
    aicontrol.drive_x (2)
    aicontrol.drive_z (const.NAV_ROLL_DEPTH)
    nav_condition = path_prac_mission.find_nav ()
    navigate_mission.run (nav_condition)

    aicontrol.drive_z (const.PING_DETECTING_DEPTH)
    pingping = PingerMission()

    aicontrol.turn_yaw_relative (45)
    pingping.run()
    aicontrol.drive_z (const.FINISH)

    aicontrol.stop (5)
    print 'FINISH !!!'
