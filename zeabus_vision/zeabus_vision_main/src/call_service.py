#!/usr/bin/env python
import cv2
import rospy
from zeabus_vision_srv_msg.msg import *
from zeabus_vision_srv_msg.srv import *
from std_msgs.msg import String
from vision_lib import *


def call_service(mission, request):
    print_result('wait service')

    if mission == 'bouy':
        vision_srv = vision_srv_bouy
    elif mission == 'navigate':
        vision_srv = vision_srv_navigate
    elif mission == 'squid':
        vision_srv = vision_srv_squid
    elif mission == 'bin':
        vision_srv = vision_srv_default

    serviceName = 'vision_' + str(mission)
    rospy.wait_for_service(serviceName)
    print_result('service start')

    call = rospy.ServiceProxy(serviceName, vision_srv)

    while not rospy.is_shutdown():
        res = call(String(mission), String(request))
        print_result('')
        print res
        rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('call_service')
    mission = 'bouy'
    request = 'a'
    call_service(mission, request)
