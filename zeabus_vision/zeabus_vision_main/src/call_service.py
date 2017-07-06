#!/usr/bin/env python
import cv2
import rospy
from zeabus_vision_srv_msg.msg import *
from zeabus_vision_srv_msg.srv import *
from std_msgs.msg import String
if __name__ == '__main__':
    rospy.init_node('call_service')
    # serviceName = 'vision_bouy'
    serviceName = 'vision_navigate'
    binsrv = 'vision_bin'
    print('wait service')
    rospy.wait_for_service(binsrv)
    print('service start')

    call = rospy.ServiceProxy(binsrv, vision_srv_default)
    # call = rospy.ServiceProxy(serviceName, vision_srv_bouy)
    while not rospy.is_shutdown():
        res = call(String('Navigate'), String('nocover'))
        print res
        rospy.sleep(0.1)
