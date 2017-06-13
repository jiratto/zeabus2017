#!/usr/bin/env python
import cv2
import numpy as np
import math
import rospy


def range_str2list(str):
    str = str.split(',')
    return np.array([int(str[0]), int(str[1]), int(str[2])],np.uint8)


def getColor(color, camera):
    # Navigate
    color_list_down = ['orange', 'white', 'yellow', 'red']
    color_list_top = ['orange', 'white', 'yellow', 'red']
    if camera == 'down':
        for c in color_list_down:
            if color == c:
                lower = rospy.get_param('/color_range/color_down/lower_' + c)
                upper = rospy.get_param('/color_range/color_down/upper_' + c)
    else:
        for c in color_list_top:
            if color == c:
                lower = np.array(rospy.get_param(
                    '/color_range/color_top/lower_' + c), np.uint8)
                upper = np.array(rospy.get_param(
                    '/color_range/color_top/upper_' + c), np.uint8)
    lower = range_str2list(lower)
    upper = range_str2list(upper)
    return lower, upper


def find_shape(cnt, req):
    peri = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
    if len(approx) == 3 and req == 'tri':
		return True
    elif len(approx) == 4 and req == 'rect':
		return True
    elif len(approx) >= 25 and req == 'cir':
	    return True
    elif len(approx) == req:
        return True
	return False

def equalization(frame):
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(img)

    equ_s = cv2.equalizeHist(s)
    equ_v = cv2.equalizeHist(v)

    equ = cv2.merge((h, equ_s, equ_v))

    # h, s, v = cv2.split(result_hsv)
    print h.max(), s.max(), v.max()

    return equ


def clahe(img):
    # ____________________________ #

    img = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)
    L, a, b = cv2.split(img)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    L = clahe.apply(L)

    result = cv2.merge((L, a, b))
    result_bgr = cv2.cvtColor(result, cv2.COLOR_Lab2BGR)
    result_hsv = cv2.cvtColor(result_bgr, cv2.COLOR_BGR2HSV)
    return result_hsv


def stretching(img):

    b, g, r = cv2.split(img)
    b -= b.min()
    b *= int(round(255.0 / (b.max() - b.min())))
    g -= g.min()
    g *= int(round(255.0 / (g.max() - g.min())))
    # r -= r.min()
    # r *= 255 /( r.max() - r.min())

    img = cv2.merge((b, g, r))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(img)

    s -= s.min()
    s *= int(round(255.0 / (s.max() - s.min())))

    v -= v.min()
    v *= int(round(255.0 / (v.max() - v.min())))

    img = cv2.merge((h, s, v))
    img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)

    return img


def cut_frame_top(img):
    h, w = img.shape

    for i in range(0, h / 4):
        for j in range(0, w):
            if (img[i, j] == 255):
                img[i, j] = 0
    return img


def cut_frame_down(img):
    h, w = img.shape

    for i in range(h * 3 / 4, h):
        for j in range(0, w):
            if (img[i, j] == 255):
                img[i, j] = 0
    return img


def mask_longerline(img):
    h, w = img.shape
    d = dmax = row = 0

    for i in range(h / 4, h):
        d = 0
        for j in range(0, w):
            if(img[i, j] == 255):
                d += 1
        if(d > dmax):
            row = i

    result = np.zeros((h, w), dtype=np.uint8)
    start = row - 50
    last = row + 50
    if last > h:
        last = h
    if start < 0:
        start = 0
    for i in range(start, last):
        for j in range(0, w):
            if img[i, j] == 255:
                result[i, j] = 255

    return result


def Oreintation(moment):
    tmp = pow((moment['mu20'] - moment['mu02']), 2)
    tmp = math.sqrt(tmp + (4 * pow(moment['mu11'], 2)))

    k = moment['mu02'] - moment['mu20']

    l = 2 * moment['mu11']

    rad_maj = math.atan2(k + tmp, l)
    rad_min = math.atan2(k - tmp, l)
    return rad_maj, rad_min


def nothing(variable):
    pass


def rebalance(image):
    h, w, ch = image.shape
    m = cv2.mean(image)
    img = np.zeros((h, w), dtype=np.float)
    img = image.copy()
    b, g, r = cv2.split(img)

    gg = m[0] * 1.0 / m[1] * 1.0
    rr = m[0] * 1.0 / m[2] * 1.0

    matrix_b = np.zeros((h, w), dtype=np.float32)
    matrix_g = np.zeros((h, w), dtype=np.float32)
    matrix_r = np.zeros((h, w), dtype=np.float32)
    matrix_b.fill(1.0)
    matrix_g.fill(gg)
    matrix_r.fill(rr)

    b = b * matrix_b
    g = g * matrix_g
    r = r * matrix_r

    result = cv2.merge((b, g, r))
    result_uint = result.astype(np.uint8)

    # ____________________________ #

    img = cv2.cvtColor(result_uint, cv2.COLOR_BGR2Lab)
    L, a, b = cv2.split(img)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    L = clahe.apply(L)

    result = cv2.merge((L, a, b))
    result_bgr = cv2.cvtColor(result, cv2.COLOR_Lab2BGR)
    result_hsv = cv2.cvtColor(result_bgr, cv2.COLOR_BGR2HSV)
    # h, s, v = cv2.split(result_hsv)
    # print h.max(),s.max(),v.max()
    return result_hsv

# def cut_contours(hh,ww,c):
# 	if cv2.contourArea(c):
# 		print '11'
# 		x,y,w,h = cv2.boundingRect(c)
# 		print x, x+w, y, y+h
# 		if x <= 1 or x+w >= ww :
# 			return False
# 		return True
# 	return False


def process_img_down(img):
    bgr = stretching(img.copy())
    hsv = clahe(bgr.copy())
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    median = cv2.medianBlur(bgr.copy(), 7)
    hsv = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)

    return hsv


def process_img_top(img):
    bgr = stretching(img.copy())
    # hsv = clahe(bgr.copy())
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return hsv


def shrinking_hsv(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    hmin = h.min()
    hmax = h.max()
    hl = 180
    hs = ((h - hmin) / (hmax - hmin)) * (hl - 1)

    smin = s.min()
    smax = s.max()
    sl = 255
    ss = ((s - smin) / (smax - smin)) * (sl - 1)

    vmin = v.min()
    vmax = v.max()
    vl = 255
    vs = ((v - vmin) / (vmax - vmin)) * (vl - 1)

    result = cv2.merge((hs, ss, vs))
    return result


def mean_staturation(hsv):
    h, s, v = cv2.split(hsv)
    mean = cv2.mean(v)
    print('mean: {0}'.format(mean))
