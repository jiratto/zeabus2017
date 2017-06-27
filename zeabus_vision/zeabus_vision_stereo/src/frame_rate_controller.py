#!/usr/bin/env python
import cv2
import serial
import rospy
import dynamic_reconfigure.client
ser = None
client = None
node = None


def get_param(param):
    global node
    return rospy.get_param(str(node) + str(param), False)


def init_serial(port, baud_rate):
    global ser
    ser = serial.Serial('/dev/ttyACM0', 115200)
    if not ser:
        ser.open()


def set_frame_rate(frameRate):
    global node, client, ser
    print('set_frame_rate')
    print(frameRate)
    ser.write('set ' + str(frameRate) + ''.encode('utf-8'))
    params = {str('frame_rate'): frameRate}
    config = client.update_configuration(params)


def nothing(variable):
    pass


def main():
    global node, client, ser
    node = '/ueye_cam_nodelet_leftcam_top'
    name = 'frame_rate'
    frameRate = int(get_param('frame_rate'))
    port = '/dev/ttyACM0'
    baud_rate = 115200
    client = dynamic_reconfigure.client.Client(node)
    print('init client')

    cv2.namedWindow(name, flags=cv2.WINDOW_NORMAL)
    cv2.moveWindow(name, 20, 20)
    cv2.resizeWindow(name, 600, 20)
    cv2.createTrackbar('value', name, 1, 30, nothing)
    cv2.setTrackbarPos('value', name, frameRate)

    init_serial(port, baud_rate)
    print('init serial')
    while not rospy.is_shutdown():
        value = int(cv2.getTrackbarPos('value', name))
        if value != frameRate:
            cv2.setTrackbarPos('value', name, value)
            frameRate = value
            set_frame_rate(frameRate)

        if value == 0:
            frameRate = 1
            cv2.setTrackbarPos('value', name, frameRate)
            set_frame_rate(frameRate)

        print('set frame rate')
        print(frameRate)
        key = cv2.waitKey(1) & 0xff
        if key == ord('q'):
            break
        rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('frame_rate_controller')
    main()
