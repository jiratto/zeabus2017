#!/usr/bin/env python

import rospy
from modbus_ascii_ros.srv import IOCommand

class Hardware:

    def __init__ (self):
        # rospy.init_node('hardware_service', anonymous=True)
        print 'init node hardware'

    def command (self, eq, status):
        if status == 'fire' or status == 'grab' or status == 'drop': 
            srv_name = '/io_and_pressure/IO_ON'
        elif status == 'close' or status == 'leave':
            srv_name = 'io_and_pressure/IO_OFF'
        else:
            return
        rospy.wait_for_service(srv_name)
        srv = rospy.ServiceProxy(srv_name, IOCommand)
        print 'service complete'

        equipment = ['fire_left', 'one', 'two', 'drop_left', 'fire_right', 'five', 'drop_right', 'seven']
        result = srv(equipment.index(eq))

        print result

if __name__ == '__main__':
    hw = Hardware()
    print 'hardware'

    # hw.command ('fire_left', 'fire')
    # hw.command ('fire_left', 'close')

    # hw.command ('one', 'fire')
    # hw.command ('one', 'close')

    # hw.command ('two', 'fire')
    # hw.command ('two', 'close')

    # hw.command ('drop_left', 'fire')
    # hw.command ('drop_left', 'close')

    # hw.command ('fire_right', 'fire')
    hw.command ('fire_right', 'close')

    # hw.command ('five', 'fire')
    # hw.command ('five', 'close')

    # hw.command ('drop_right', 'fire')
    # hw.command ('drop_right', 'close')

    # hw.command ('seven', 'fire')
    # hw.command ('seven', 'close')

