#!/usr/bin/env python
import rospy
from std_msgs.msg import String 
from zeabus_vision_srv_msg.srv  import vision_srv_default
if __name__ == '__main__':
    service = rospy.ServiceProxy('vision',vision_srv_default)
    while not rospy.is_shutdown():
        t = service(String('path'),String('orange')) 
        

