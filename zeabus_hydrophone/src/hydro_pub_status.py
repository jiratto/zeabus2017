#!/usr/bin/env python
from std_msgs.msg import Bool
import rospy
if __name__ == '__main__':
	rospy.init_node('hydro_pub')
	pub = rospy.Publisher('/hydro_status', Bool, queue_size =  1)
	for i in range (0,3):
		print i
		pub.publish(True)
		rospy.sleep(0.1)
