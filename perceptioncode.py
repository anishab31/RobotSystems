#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import math
import rospy
import numpy as np
from threading import RLock, Timer

from std_srvs.srv import *
from sensor_msgs.msg import Image

from sensor.msg import Led
from object_tracking.srv import *
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from kinematics import ik_transform

from armpi_fpv import PID
from armpi_fpv import Misc
from armpi_fpv import bus_servo_control

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


class Perception(object):
    range_rgb = {
        'red': (0, 0, 255),
        'blue': (255, 0, 0),
        'green': (0, 255, 0),
        'black': (0, 0, 0),
        'white': (255, 255, 255),
    }

    size = (320, 240)


    def __init__(self):
        #motion instructions
         self.x_pid = PID.PID(P=0.1, I=0.00, D=0.008)
         self.y_pid = PID.PID(P=0.00001, I=0, D=0)
         self.z_pid = PID.PID(P=0.005, I=0, D=0)

     def reset(self):
         For motion instructions maybe
         self.x_pid = PID.PID(P=0.1, I=0.00, D=0.008)
         self.y_pid = PID.PID(P=0.00001, I=0, D=0)
         self.z_pid = PID.PID(P=0.005, I=0, D=0)

    def findcube_color(self, img, target_color_range, start=True):  #colored cubes
         if start = True:
            self.reset()

        frame_mask = self.clean_build_color_mask(img, target_color_range)
        contours = self.get_contour_by_mask(frame_mask)
        contour, area_max = self.get_max_contour(contours)
        if contour is not None and area_max is not None:
            img_draw, centers = self.get_contour_box(contour, img)
            return img_draw, centers
        return img, None


    def color_mask(self, img, target_color_range):
        # make a copy of the image, and get the size
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_lab = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空
        frame_mask = cv2.inRange(frame_lab, tuple(target_color_range['min']),
                                 tuple(target_color_range['max']))  # 对原图像和掩模进行位运算
        return frame_mask

    def get_contour_by_mask(self, mask):
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
        return contours

    def max_contour(self, contours):
        # get the max contour
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in self.contours:  # 历遍所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 10:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                    area_max_contour = c
        if contour_area_max > 100:
            return area_max_contour, contour_area_max  # 返回最大的轮廓
        return None, None

    def contour_box(self, contour, img):


        img_h, img_w = img.shape[:2]
        (center_x, center_y), radius = cv2.minEnclosingCircle(contour)  # 获取最小外接圆
        center_x = int(Misc.map(center_x, 0, self.size[0], 0, img_w))
        center_y = int(Misc.map(center_y, 0, self.size[1], 0, img_h))
        radius = int(Misc.map(radius, 0, self.size[0], 0, img_w))
        if radius > 100:
            return img, None

        rect = cv2.minAreaRect(contour)
        box = np.int0(cv2.boxPoints(rect))
        cv2.putText(img,"color:"+[target_color)


        return img, (center_x, center_y)


