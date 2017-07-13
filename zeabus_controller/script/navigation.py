#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
import tf
import math
from zeabus_controller.srv import navigation_srv
position = [0, 0, 0, 0, 0, 0, 0]


def twopi(rad):
    if rad <= 0:
        return abs(rad)
    else:
        return 2 * math.pi - rad


def get_position(data):
    global position

    pose = data.pose.pose
    temp = (pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)
    angular = tf.transformations.euler_from_quaternion(temp)
    position[0] = pose.position.x
    position[1] = pose.position.y
    position[2] = pose.position.z
    position[3] = angular[0]
    position[4] = angular[1]
    position[5] = angular[2]
    position[6] = math.degrees(position[5])


def drive(x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
    pubTwist = rospy.Publisher('/zeabus/cmd_vel', Twist, queue_size=10)
    t = Twist()
    t.linear.x = x
    t.linear.y = y
    t.linear.z = z
    t.angular.x = roll
    t.angular.y = pitch
    t.angular.z = yaw
    for i in xrange(5):
        print t
        pubTwist.publish(t)
        rospy.sleep(0.05)


def turn_yaw_rel(degree):
    global position
    turn_yaw_abs(degree + position[6])
    print ('turn yaw relative: ', degree)


def turn_yaw_abs(degree):
    turnYawAbs = rospy.Publisher(
        '/fix/abs/yaw', Float64, queue_size=10)
    rad = math.radians(degree)
    rad = Float64(rad)
    for i in xrange(5):
        turnYawAbs.publish(rad)
        rospy.sleep(0.04)
    print ('turn yaw absolute: ', degree)


def go_to_xy(x=None, y=None, yaw=None):
    global position
    drive(0, 0, 0, 0, 0, 0)
    print ('goto')
    print (x, y)

    vx = 0.2

    if x is None:
        x = position[0]
    if y is None:
        y = position[1]
    if yaw is None:
        yaw = position[6]
    #               X
    #               |
    #               |
    # -y ----------------------- y
    #               |
    #               |
    #               -x
    start_x = position[0]
    start_y = position[1]
    while not rospy.is_shutdown():
        begin_x = position[0]
        begin_y = position[1]

        scale_x = abs(start_x - x)
        scale_y = abs(start_y - y)
        scale = scale_x**2 + scale_y**2

        delta_x = x - begin_x
        delta_y = -(y - begin_y)
        # print('x y')
        # print(x, y)
        # print('begin')
        # print (begin_x, begin_y)
        # print('deelta')
        # print(delta_x, delta_y)
        #######################################################################
        begin_rad = math.atan2(delta_x, delta_y)
        begin_deg = math.degrees(begin_rad)
        if begin_deg < 0:
            begin_deg += 360
        print('begin deg')
        print begin_deg

        rad = position[5]
        # print rad
        deg = math.degrees(rad)
        if deg < 0:
            deg += 360
        deg += 90
        deg %= 360
        print 'deg'
        print deg
        res_deg = [deg - begin_deg - 360,
                   begin_deg - deg, deg - begin_deg + 360]

        res_deg = min(res_deg, key=abs)
        # print('res degree')
        # print(res_deg)
        turn_yaw_rel(res_deg)
        rospy.sleep(4)
        print 'drive'
        #######################################################################
        dis = (begin_x - x)**2 + (begin_y - y)**2
        if begin_y < y:
            vy = dis / scale
        else:
            vy = -dis / scale
        if abs(begin_y - y) <= 0.5:
            vy = 0
        vx = dis / scale
        print('vx: ') + str(vx) + ' ' + ('vy: ') + str(vy)
        drive(vx + 0.1, vy, 0, 0, 0, 0)
        rospy.sleep(0.2)
        print 'fin drive'
        print (dis)
        if dis <= (0.25**2):
            print('stop')
            drive(0, 0, 0, 0, 0, 0)
            rospy.sleep(2)
            break
        print position
    turn_yaw_abs(yaw)
    print ('finish')
    print position
    # m = Bool()
    # m.success = True
    return True


def navigation_callback(req):
    # req = msg.req.data
    print('request: ') + str(req)

    x = req.x
    y = req.y
    yaw = req.yaw
    return go_to_xy(x, y, yaw)

if __name__ == '__main__':
    rospy.init_node('navigation')
    rospy.Subscriber('/auv/state', Odometry, get_position)
    # rospy.Subscriber('/syrena/state', Odometry, get_position)

    rospy.Service('navigation', navigation_srv(), navigation_callback)
    rospy.spin()
