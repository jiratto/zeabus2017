#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
x = None
pub = None


def callback(msg):
    global x
    x = msg.pose.pose.position.x


def drive_x_rel(dis):
    global pub
    while x is None:
        rospy.sleep(0.01)

    start_x = x
    print start_x
    t = Twist()
    t.linear.x = 0.6
    t.linear.y = 0
    t.linear.z = 0
    t.angular.x = 0
    t.angular.y = 0
    t.angular.z = 0

    while not rospy.is_shutdown():
        pub.publish(t)
        if x >= start_x + dis:
            t.linear.x = 0
            print('stop')
            print x

            for i in xrange(3):
                pub.publish(t)
                rospy.sleep(5)
            break
        rospy.sleep(0.5)
if __name__ == '__main__':
    rospy.init_node('drive_x')
    rospy.Subscriber('/auv/state', Odometry, callback)
    pub = rospy.Publisher('/zeabus/cmd_vel', Twist, queue_size=1)
    # rospy.sleep(3)
    drive_x_rel(1)
