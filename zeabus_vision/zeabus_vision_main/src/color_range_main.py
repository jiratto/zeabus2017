#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
import rospy
from sensor_msgs.msg import CompressedImage
from vision_lib import *

pixel = {}
pixel['x'], pixel['y'] = -1, -1
click = False
is_mask = False
img = None
hsv = None
wait = False
width = int(1152 / 3)
height = int(870 / 3)
cameraPos = 'down'
mission = None


class window:
    global width, height, hsv, cameraPos

    def __init__(self):
        self.size = 250
        self.x = width / 3
        self.y = 0
        self.lower = {}
        self.upper = {}
        self.lower_tmp = {}
        self.upper_tmp = {}
        self.select = {}
        self.path = rospkg.RosPack().get_path('zeabus_vision')

    def create(self, window_name):
        for name in window_name:
            cv2.namedWindow(name, flags=cv2.WINDOW_NORMAL)
            cv2.moveWindow(name, self.x + self.x / 5, self.y + self.y / 5)
            cv2.resizeWindow(name, self.size, self.size)
            self.update_position()
            self.create_range(name)

    def update_position(self):
        self.y += self.size
        if self.y + self.size >= height:
            self.x += self.size
            self.y = 20

    def create_range(self, name):
        lower_param, upper_param = self.get_param(name)
        self.lower[name] = [lower_param]
        self.upper[name] = [upper_param]
        self.lower_tmp[name] = []
        self.upper_tmp[name] = []
        self.select[name] = False

    def push_range(self, name, lower, upper):
        self.lower[name].append(lower)
        self.upper[name].append(upper)

    def get_range(self, name):
        return self.lower[name][-1], self.upper[name][-1]

    def undo_range(self, name):
        if len(self.lower[name]) > 0:
            self.lower_tmp[name].append(self.lower[name][-1])
            self.upper_tmp[name].append(self.upper[name][-1])
            self.lower[name].pop()
            self.upper[name].pop()
            set_trackbar(self.lower[name][-1], self.upper[name][-1])
            print('undo')
        else:
            print('cannot undo')

    def redo_range(self, name):
        if len(self.lower_tmp[name]) > 0:
            self.lower[name].append(self.lower_tmp[name][-1])
            self.upper[name].append(self.upper_tmp[name][-1])
            self.lower_tmp[name].pop()
            self.upper_tmp[name].pop()
            set_trackbar(self.lower[name][-1], self.upper[name][-1])
            print('redo')
        else:
            print('cannot redo')

    def reset_range(self, name):
        self.lower[name].append([179, 255, 255])
        self.upper[name].append([0, 0, 0])
        set_trackbar(self.lower[name][-1], self.upper[name][-1])
        print('reset')

    def show_image(self, window_name):
        for name in window_name:
            result = cv2.inRange(hsv, np.array(self.lower[name][-1], np.uint8),
                                 np.array(self.upper[name][-1], np.uint8))
            cv2.imshow(name, result)

    def range_str2list(self, str):
        str = str.split(',')
        return [int(str[0]), int(str[1]), int(str[2])]

    def range_list2str(self, list):
        seq = (str(list[0]), str(list[1]), str(list[2]))
        ch_join = ','
        return ch_join.join(seq)

    def get_param(self, name):
        self.param_lower = rospy.get_param(
            'color_range/color_' + cameraPos + '/lower_' + name, '179,255,255')
        self.param_upper = rospy.get_param(
            'color_range/color_' + cameraPos + '/upper_' + name, '0,0,0')
        self.param_lower = self.range_str2list(self.param_lower)
        self.param_upper = self.range_str2list(self.param_upper)
        return self.param_lower, self.param_upper

    def save(self):
        for name in self.lower:
            if(name == 'mask'):
                continue
            rospy.set_param(
                '/color_range/color_' + cameraPos + '/lower_' + name, self.range_list2str(self.lower[name][-1]))
            rospy.set_param(
                '/color_range/color_' + cameraPos + '/upper_' + name, self.range_list2str(self.upper[name][-1]))

        f = open(self.path + "/params/color_" + cameraPos + ".yaml", "w")
        x = self.genyaml()
        f.write(x)
        f.close()

        print 'save'

    def genyaml(self):
        tmp = " color_" + cameraPos + ":\n"
        for name in self.lower:
            if(name == 'mask'):
                continue
            tmp += " " + " upper_" + name + ": '" + self.range_list2str(self.upper[name][-1]) + "'\n\n" +\
                " " + " lower_" + name + ": '" + \
                self.range_list2str(self.lower[name][-1]) + "'\n\n"
        return tmp


def camera_callback(msg):
    global img, wait, hsv, width, height, mission
    if wait == False:
        arr = np.fromstring(msg.data, np.uint8)
        img_data = cv2.resize(cv2.imdecode(arr, 1), (width, height))
        if
        img = preprocess_squid(img_data)
        # img = preprocess_navigate(img_data)
        # img = preprocess_bouy(img_data)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # blur = cv2.bilateralFilter(img, 9, 75, 75)
        # cla = clahe(img)
        # hsv1 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # hsv = equalization_hsv(hsv1)

        # bgr = equalization_bgr(img)
        # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        bgr = preprocess_navigate(img)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # hsv1 = preprocess_squid(img)
        # hsv = cv2.cvtColor(hsv1, cv2.COLOR_BGR2HSV)

        # image = cv2.imread('table.png')
        # image = cv2.resize(image, (width, height))
        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


def draw_circle(event, x, y, flags, param):
    global pixel, click
    if event == cv2.EVENT_LBUTTONDOWN:
        click = True
        pixel['x'], pixel['y'] = x, y


def has_color(window_name, k):
    for name in window_name:
        if k == ord(name[0]) and k != ord('m'):
            return name, True
    return None, False


def set_trackbar(lower, upper):
    [hmin, smin, vmin], [hmax, smax, vmax] = lower, upper
    cv2.setTrackbarPos('Hmin', 'image', hmin)
    cv2.setTrackbarPos('Smin', 'image', smin)
    cv2.setTrackbarPos('Vmin', 'image', vmin)
    cv2.setTrackbarPos('Hmax', 'image', hmax)
    cv2.setTrackbarPos('Smax', 'image', smax)
    cv2.setTrackbarPos('Vmax', 'image', vmax)


def get_trackbar():
    Hmin = cv2.getTrackbarPos('Hmin', 'image')
    Smin = cv2.getTrackbarPos('Smin', 'image')
    Vmin = cv2.getTrackbarPos('Vmin', 'image')
    Hmax = cv2.getTrackbarPos('Hmax', 'image')
    Smax = cv2.getTrackbarPos('Smax', 'image')
    Vmax = cv2.getTrackbarPos('Vmax', 'image')
    lower = [Hmin, Smin, Vmin]
    upper = [Hmax, Smax, Vmax]
    return lower, upper


def compare_range(l, u, l1, u1):
    return not (l == l1 and u == u1)


def select_color():
    global pixel, img, wait, hsv, click, is_mask, width, height
    window_name = ['mask', 'red', 'orange',
                   'white', 'yellow', 'black', 'green ,']

    cv2.namedWindow('image_bgr', flags=cv2.WINDOW_NORMAL)
    cv2.moveWindow('image_bgr', 400, 400)
    cv2.resizeWindow('image_bgr', (width / 3), (height / 3))
    cv2.createTrackbar('gamma', 'image_bgr', 0, 100, nothing)
    cv2.setTrackbarPos('gamma', 'image_bgr', 1)

    cv2.namedWindow('image', flags=cv2.WINDOW_NORMAL)
    cv2.moveWindow('image', 20, 20)
    cv2.resizeWindow('image', (width / 3), height)
    cv2.createTrackbar('Hmin', 'image', 0, 179, nothing)
    cv2.createTrackbar('Smin', 'image', 0, 255, nothing)
    cv2.createTrackbar('Vmin', 'image', 0, 255, nothing)
    cv2.createTrackbar('Hmax', 'image', 0, 179, nothing)
    cv2.createTrackbar('Smax', 'image', 0, 255, nothing)
    cv2.createTrackbar('Vmax', 'image', 0, 255, nothing)
    cv2.createTrackbar('m <-> c', 'image', 0, 2, nothing)
    cv2.createTrackbar('shoot_x', 'image', 0, width, nothing)
    cv2.createTrackbar('shoot_y', 'image', 0, height, nothing)
    set_trackbar([179, 255, 255], [0, 0, 0])
    cv2.setTrackbarPos('shoot_x', 'image', int(width / 2))
    cv2.setTrackbarPos('shoot_y', 'image', int(height / 2))
    cv2.setMouseCallback('image', draw_circle)

    w = window()
    w.create(window_name)

    while(img is None):
        rospy.sleep(0.01)

    while not rospy.is_shutdown():

        key = cv2.waitKey(1) & 0xff
        if key == ord('p') and wait == False and not click:
            wait = True
        elif key == ord('p') and wait == True and not click:
            wait = False

        name, status = has_color(window_name, key)
        lower, upper = w.get_range('mask')
        lower_bar, upper_bar = get_trackbar()

        if click and is_mask:
            h, s, v = hsv[pixel['y'], pixel['x']]
            [hl, sl, vl], [hu, su, vu] = w.get_range('mask')
            lower_current = [min(h, hl), min(s, sl), min(v, vl)]
            upper_current = [max(h, hu), max(s, su), max(v, vu)]
            w.push_range('mask', lower_current, upper_current)
            set_trackbar(lower_current, upper_current)
        elif compare_range(lower, upper, lower_bar, upper_bar):
            w.push_range('mask', lower_bar, upper_bar)
        elif status:
            if w.select[name]:
                lower_current, upper_current = w.get_range('mask')
                w.push_range(name, lower_current, upper_current)
                cv2.setTrackbarPos('m <-> c', 'image', 2)
                is_mask = False
            else:
                lower_current, upper_current = w.get_param(name)
                w.push_range('mask', lower_current, upper_current)
                set_trackbar(lower_current, upper_current)
                cv2.setTrackbarPos('m <-> c', 'image', 0)
                is_mask = True
            w.select[name] = not w.select[name]
        elif is_mask:
            if key == ord('z'):
                w.undo_range('mask')
            elif key == ord('x'):
                w.redo_range('mask')
            elif key == ord('c'):
                w.reset_range('mask')
        elif key == ord('s'):
            w.save()
        elif key == ord('q'):
            break
        x = cv2.getTrackbarPos('shoot_x', 'image')
        y = cv2.getTrackbarPos('shoot_y', 'image')
        w.show_image(window_name)
        cv2.circle(hsv, (int(x), int(y)), 5, (100, 255, 255), -1)
        cv2.imshow('image', hsv)
        cv2.imshow('imageBGR', img)
        click = False
        status = False
    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('color_range_main')
    cameraPos = rospy.get_param('color_range/cameraPos', 'down')
    cameraTopic = rospy.get_param('color_range/cameraTopic',
                                  '/rightcam_bottom/image_raw/compressed')
    mission = rospy.get_param('color_range/mission', 'default')
    print('topic: ' + str(cameraTopic) + ' camera: ' +
          str(cameraPos) + ' mission: ' + str(mission))
    rospy.Subscriber(cameraTopic, CompressedImage, camera_callback)
    select_color()
