#!/usr/bin/env python
import cv2
import rospy
from zeabus_vision_srv_msg.msg import *
from zeabus_vision_srv_msg.srv import *
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('call_service')
    serviceName = 'vision_navigate'
    # serviceName = 'vision_navigate'
    # serviceName = 'vision_squid'
    # binsrv = 'vision_bin'
    print('wait service')
    rospy.wait_for_service(serviceName)
    print('service start')

    # call = rospy.ServiceProxy(serviceName, vision_srv_navigate)
    # call = rospy.ServiceProxy(serviceName, vision_srv_bouy)
    call = rospy.ServiceProxy(serviceName, vision_srv_navigate)
    while not rospy.is_shutdown():
        res = call(String(''), String('top'))
        print res

        rospy.sleep(0.1)