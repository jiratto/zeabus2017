
#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from zeabus_hydrophone.srv import hydro_info
from zeabus_hydrophone.msg import hydro_msg
from AIControl import AIControl
import math

class PingerMission(object):
    def __init__(self):
        print 'pinger init'
        ### service hydrophone ###
        # rospy.wait_for_service('hydro')
        # self.srvPing = rospy.ServiceProxy('hydro',hydro_info)
        # print 'set up service complete'
        self.aicontrol = AIControl()
        # self.data=hydro_msg()
        self.hy = hydro_msg()
        ### subscribe hydrophone ###
        rospy.Subscriber ('/hydro', hydro_msg, self.listening)
    
    def convert(self, azi):
        print 'covert degree to turn'
        azi = -azi
        real = azi - 45
        if real < -180:
            real = real + 360
        return real

    def listening(self, data):
        self.hy = data

    def diff(sefl, now, want):
        if abs(now - want) > math.pi:
            return abs(now - want) - 2*math.pi
        else:
            return abs(now - want)

               
    def run(self):
        pinger = self.hy
        real_degree = self.convert(pinger.azi)
        print pinger.azi
        print real_degree
        self.aicontrol.turnyawrel(math.radians(real_degree))
        self.aicontrol.drive_x (1)
        print 'drive'
        rospy.sleep(2)

if __name__=='__main__':
    print 'hydrophone'
    rospy.init_node('ping_ai')
    ping=PingerMission()
    while not rospy.is_shutdown():
    	ping.run()
        rospy.spin()
