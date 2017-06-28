#!/usr/bin/python
import rospy
import os
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo

sec1 = 1
sec2 = 0


def callback(msg):
    global sec1
    sec1 = int(msg.header.stamp.nsecs)


def callback1(msg):
    global sec2
    sec2 = int(msg.header.stamp.nsecs)

if __name__ == '__main__':
    rospy.init_node('test_pub_time')
    rospy.Subscriber("/rightcam_top/camera_info", CameraInfo, callback)
    rospy.Subscriber("/leftcam_top/camera_info", CameraInfo, callback1)
    ct = 0
    while not rospy.is_shutdown():
        diff = sec1 - sec2
        print(sec1)
        print(sec2)
        print(diff)
        print(ct)
        if diff == 0:
            ct += 1
        os.system('clear')
