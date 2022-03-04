#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import math
import rospy
import numpy as np

from kinematics import ik_transform

from armpi_fpv import PID
from armpi_fpv import Misc
from armpi_fpv import bus_servo_control
from sorting import *
from tracking import *



#movement class
class Motion(object):
    size = (320, 240)

    def __init__(self, joints_pub):

        # For motion instructions maybe
        self.x_pid = PID.PID(P=0.06, I=0.005, D=0)  # pid初始化
        self.y_pid = PID.PID(P=0.00001, I=0, D=0)
        self.z_pid = PID.PID(P=0.00003, I=0, D=0)

        self.ik = ik_transform.ArmIK()

        self.joints_pub = joints_pub

    def reset(self):
        # reset at start point
        self.x_pid = PID.PID(P=0.06, I=0.005, D=0)  # pid初始化
        self.y_pid = PID.PID(P=0.00001, I=0, D=0)
        self.z_pid = PID.PID(P=0.00003, I=0, D=0)

    def get_cube(self):  #from sorting.py, "pick" portion of code
        pick()

    def drop_cube(self): #from sorting.py
        place()


    def get_new_movement(self, img, x_dis, y_dis, z_dis, (center_x, center_y), area_max): #basic motion to for next cube
        img_h, img_w = img.shape[:2]
        self.x_pid.SetPoint = img_w / 2.0
        self.x_pid.update(center_x)
        dx = self.x_pid.output
        x_dis += int(dx)

        x_dis = 200 if x_dis < 200 else x_dis
        x_dis = 800 if x_dis > 800 else x_dis

        self.y_pid.SetPoint = 900
        if abs(area_max - 900) < 50:
            area_max = 900
        self.y_pid.update(area_max)
        dy = self.y_pid.output
        y_dis += dy
        y_dis = 0.12 if y_dis < 0.12 else y_dis
        y_dis = 0.25 if y_dis > 0.25 else y_dis

        if abs(center_y - img_h / 2.0) < 20:
            self.z_pid.SetPoint = center_y
        else:
            self.z_pid.SetPoint = img_h / 2.0

        self.z_pid.update(center_y)
        dy = self.z_pid.output
        z_dis += dy

        z_dis = 0.22 if z_dis > 0.22 else z_dis
        z_dis = 0.17 if z_dis < 0.17 else z_dis

        return x_dis, y_dis, z_dis

    def move_with_ik(self, x_dis, y_dis, z_dis):
        target = self.ik.setPitchRanges((0, round(y_dis, 4), round(z_dis, 4)), -90, -85, -95)
        if target:
            servo_data = target[1]
            #joints pub not resolved, needs re-checking
            bus_servo_control.set_servos(self.joints_pub, 20, (
                (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, x_dis)))


if __name__ == '__main__':
    motionControl = Motion()