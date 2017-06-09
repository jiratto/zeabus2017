#!/usr/bin/env python

import rospy
import math
import tf
import Queue as Queue
from geometry_msgs.msg import Twist,TwistStamped,Pose,PoseStamped,Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64,Bool
from geometry_msgs.msg import Quaternion
from controller.srv import drive_x

class AIControl():

    # _instance = None
    # def __new__(cls, *args, **kwargs):
    #     if not cls._instance:
    #         cls._instance = super(Singleton, cls).__new__(cls, *args, **kwargs)
    #     return cls._instance

    # def __init__ (self):
    #     self.is_at_fix_position = True
    #     self.err = 0.1
    #     self.pose = Pose()
    #     self.auv = [0,0,0,0,0,0]
    #     self.goal = [-999,-999,-999]
    #     self.stopt = True
    #     rospy.Subscriber ('/auv/state', Odometry, self.set_position)
    #     rospy.Subscriber ('/controller/is_at_fix_position', Bool, self.posi)
    #     rospy.Subscriber ('/controller/is_at_fix_orientation', Bool, self.ck_turn)
        
    #     self.command = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    #     self.turn_yaw_rel = rospy.Publisher('/fix/rel/yaw', Float64, queue_size = 10)
    #     self.turn_yaw_abs = rospy.Publisher('/fix/abs/yaw', Float64, queue_size = 10)
    #     self.depth = rospy.Publisher('/fix/abs/depth', Float64, queue_size = 10)
    #     self.go_to_point_publisher = rospy.Publisher('/cmd_fix_position', Point, queue_size = 10)
        
    #     rospy.wait_for_service('fix_rel_x_srv')
    #     print 'service starts drive_x'
    #     self.drive_x_srv = rospy.ServiceProxy('fix_rel_x_srv', drive_x)
    #     self.wait_for_subscriber()

    ##### start set environment #####
    # def ck_turn(self, data):
    #     self.stopt = data.data

    # def stop_turn (self):
    #     return self.stopt

    # def listToTwist (self, list):
    #     temp = Twist()
    #     temp.linear.x = list[0]
    #     temp.linear.y = list[1]
    #     temp.linear.z = list[2]
    #     temp.angular.x = list[3]
    #     temp.angular.y = list[4]
    #     temp.angular.z = list[5]
    #     return temp

    # def pub (self, tw):
    #     print "linear  x:%f y:%f z:%f"%(tw.linear.x,tw.linear.y,tw.linear.z)
    #     # print "angular x:%f y:%f z:%f"%(tw.angular.x,tw.angular.y,tw.angular.z)
    #     for i in xrange(5):
    #         self.command.publish(tw)
    #         rospy.sleep(0.05)

    # def set_position (self, data):
    #     self.pose = data.pose.pose
    #     pose = data.pose.pose
    #     tmp = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    #     ang = tf.transformations.euler_from_quaternion(tmp)
    #     self.auv[0] = pose.position.x
    #     self.auv[1] = pose.position.y
    #     self.auv[2] = pose.position.z
    #     self.auv[3] = ang[0]
    #     self.auv[4] = ang[1]
    #     self.auv[5] = ang[2]
    #     # print self.auv

    # def get_pose(self):
    #     return self.auv
    
    # def posi (self, data):
    #     self.is_at_fix_position = data.data
    
    ##### end set environment #####

    ##### move auv command #####
    # def drive (self, list):
    #     self.pub(self.listToTwist(list))

    # def stop (self, time):
    #     self.pub(self.listToTwist([0,0,0,0,0,0]))
    #     rospy.sleep(time)

    # def turn_yaw_relative (self, degree):
    #     rad = math.radians(degree)
    #     rad = Float64(rad)
    #     self.turn_yaw_rel.publish(rad)
    #     while not self.stop_turn():
    #         rospy.sleep(0.1)    
    #     print 'turn_yaw_relative'

    # def turn_yaw_absolute (self, degree):
    #     rad = math.radians (degree)
    #     rad = Float64(rad)
    #     self.turn_yaw_abs.publish(rad)
    #     rospy.sleep(0.5)
    #     while not self.stop_turn():
    #         rospy.sleep(0.1) 
    #     print 'turn_yaw_absolute'

    # def drive_z (self,z):
    #     zz = Float64(z)
    #     for i in xrange(3):
    #         # print zz
    #         self.depth.publish(zz)
    #         rospy.sleep(0.2)
    #     self.wait_reach_fix_position(timeout_threshold = 10)

    #     self.stop (1)
    #     for i in xrange(3):
    #         self.depth.publish(self.auv[2])
    #         rospy.sleep(0.2)
    #     print 'drive_z complete'

    # def drive_x (self, x):
    #     self.drive_x_srv (x, 0)
    #     print 'drive x complete'
    #     self.wait_reach_fix_position()

    # def drive_y (self, y):
    #     self.drive_x_srv (0, y)
    #     print 'drive y complete'
    #     self.wait_reach_fix_position()

    # def go_xyz (self, x, y, z):
    #     point = Point()
    #     point.x = Float64(x)
    #     point.y = Float64(y)
    #     point.z = Float64(z)
    #     self.go_to_point_publisher.publish(point);
    #     self.wait_reach_fix_position()


    ##### end move auv #####

    ##### Navigation function #####
    # def distance(self,x,y):
    #     return sum(map(lambda x,y : (x-y) **2 ,x,y))**0.5

    # def twopi(self,rad):
    #     if(rad<=0):
    #         return abs(rad)
    #     else:
    #         return 2*math.pi-rad

    # def w_yaw(self,setyaw):
    #     degi=self.twopi(self.auv[5])
    #     degf=self.twopi(setyaw)
    #     diff=(degi-2*math.pi-degf,degi-degf,2*math.pi-degf+degi)
    #     diff=min(diff,key=abs)
    #     diff*=2
    #     if(diff>=0):
    #         return abs(diff)
    #     return -abs(diff)

    # def drive_xy(self,x,y,bit):
    #     ### v : velocity
    #     print 'drive xy'
    #     v = [0,0,0,0,0,0]
    #     while not rospy.is_shutdown():
    #         print self.auv
    #         delta_y=abs(x-self.auv[1])
    #         delta_x=abs(y-self.auv[2])
    #         rad=abs(self.auv[5]-math.atan2(delta_x,delta_y))

    #         disnow = self.distance((x,y),(self.auv[0],self.auv[1]))
    #         v[0]=min(disnow,0.3)*bit

    #         if(disnow>=1):
    #             v[5]=self.w_yaw(self.delta_radians(x,y,bit))


    #         if(disnow <= self.err):
    #             self.stop()
    #             rospy.sleep(0.25)
    #             break
    #         self.drive(v)
    #         rospy.sleep(0.25)
    #     print 'Finish drive_xy'

    # def turn_yaw(self,radians):
    #     self.turn_yaw_absolute(math.degrees(radians))

    #     while not rospy.is_shutdown() and not (self.auv[5]>=radians-self.err and self.auv[5]<=radians+self.err):
    #         pass

    #     self.stop()

    # def delta_radians (self,x,y,bit):
    #     radians = math.atan2((x-self.auv[0])*bit,(y-self.auv[1])*bit)
    #     radians -= math.pi/2
    #     radians *= -1
    #     if(radians > math.pi):
    #         return radians - 2*math.pi
    #     if(radians < -math.pi):
    #         return radians + 2*math.pi
    #     return radians

    # def w_yaw(self,setyaw):
    #     degi=self.twopi(self.auv[5])
    #     degf=self.twopi(setyaw)
    #     diff=(degi-2*math.pi-degf,degi-degf,2*math.pi-degf+degi)
    #     diff=min(diff,key=abs)
    #     diff*=2
    #     if(diff>=0):
    #         return abs(diff)
    #     return -abs(diff)

    # def goto(self,x,y,z,bit):
    #     self.drive_z(z)
    #     radians = self.delta_radians(x,y,bit)
    #     print radians
    #     self.turn_yaw_relative(math.degrees(radians))
    #     print 'finish turn_yaw_relative'
    #     self.drive_xy(x,y,bit)
    ##### endNavigation function #####

    ##### image function #####
    # def is_center (self, point, xmin, xmax, ymin, ymax):
    #     if (xmin <= point[0] and point[0] <= xmax) and (ymin <= point[1] and point[1] <= ymax):
    #         return True
    #     return False

    # def adjust (self, value, m_min, m_max, p_min, p_max):
    #     if value > 0:
    #         if value > p_max : return p_max
    #         if value < p_min : return p_min
    #     elif value < 0:
    #         if value < m_min : return m_min
    #         if value > m_max : return m_max
    #     return value

    # def is_fail(self,count):
    #     if count>0:
    #         return False
    #     return True

    # def roll(self,time):
    #     q = Queue.Queue()
    #     rotate_45 = [0.3826834,0,0,0.9238795]        
    #     cmd_vel_publisher = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    #     fix_orientation_publisher = rospy.Publisher('/cmd_fix_orientation',Quaternion,queue_size=10)

    #     # calculate trajectory point and put to queue
    #     start_orientation = tf.transformations.quaternion_from_euler(0,0,self.auv[5]);
    #     x = start_orientation

    #     l = []
    #     for i in range(0,8):
    #         x = tf.transformations.quaternion_multiply(x,rotate_45)
    #         l.append(x)
    #     for i in range(0,time):
    #         for quat in l:
    #             q.put(quat)
        
    #     q.put(start_orientation)

    #     twist = Twist()
    #     twist.linear.x = 1.5;
    #     cmd = 0;
    #     last_cmd = q.qsize();
    #     print("START ROLLING")
    #     r = rospy.Rate(2)
    #     while not q.empty() and not rospy.is_shutdown():
    #         quat = q.get();
    #         cmd_vel_publisher.publish(twist)
    #         fix_orientation_publisher.publish(*quat)
    #         cmd = cmd + 1
    #         print cmd,"/",last_cmd;
    #         r.sleep()
    #     print("END OF ROLLING")
    #     twist.linear.x = 0;
    #     cmd_vel_publisher.publish(twist)

    # def wait_reach_fix_position(self, delay = 0.1,check_interval = 0.1,timeout_threshold = 10):
    #     rospy.sleep(delay)
    #     waited_time = 0;
    #     while not rospy.is_shutdown() and not self.is_at_fix_position and waited_time <timeout_threshold:
    #         waited_time += check_interval
    #         rospy.sleep(check_interval)

    # def wait_for_subscriber(self, check_interval = 0.3):
    #     fin = False
    #     while not rospy.is_shutdown() and not fin :
    #         count = 0
    #         print count
            
    #         if self.command.get_num_connections() > 0:
    #             count = count + 1
    #         if self.turn_yaw_rel.get_num_connections() > 0:
    #             count = count + 1
    #         if self.turn_yaw_abs.get_num_connections() > 0:
    #             count = count + 1
    #         if self.depth.get_num_connections() > 0:
    #             count = count + 1
    #         if self.go_to_point_publisher.get_num_connections() > 0:
    #             count = count + 1
    #         if count >= 5:
    #             fin = True
    #         if not fin:
    #             rospy.sleep(check_interval)
    #             print 'some sub not connected!'
    #             # print count


if __name__=='__main__':
    print 'AIControl'
    # rospy.init_node('ai_control', anonymous=True) #comment if run main.py
    aicontrol=AIControl()
    # aicontrol.turn_yaw_relative(90)
    # rospy.Rate(10)
    # aicontrol.goto(-3,-2,-1.5,-1)
