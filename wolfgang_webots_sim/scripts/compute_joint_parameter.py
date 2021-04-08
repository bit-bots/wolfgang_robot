#!/usr/bin/env python3

import math


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class joint_specs:
    def __init__(self, name, max_tor_12V, max_tor_14V, max_vel_12V_rpm, max_vel_14V_rpm, NT_curve_point_1,
                 NT_curve_point_2):
        self.name = name
        self.max_tor_12V = max_tor_12V
        self.max_tor_14V = max_tor_14V
        self.max_vel_12V_rpm = max_vel_12V_rpm
        self.max_vel_14V_rpm = max_vel_14V_rpm
        self.NT_curve_point_1_12V = NT_curve_point_1
        self.NT_curve_point_2_12V = NT_curve_point_2

        # convert to rad/s
        self.max_vel_12V = self.max_vel_12V_rpm * math.tau / 60
        self.max_vel_14V = self.max_vel_14V_rpm * math.tau / 60
        self.NT_curve_point_1_12V.y = self.NT_curve_point_1_12V.y * math.tau / 60
        self.NT_curve_point_2_12V.y = self.NT_curve_point_2_12V.y * math.tau / 60

        # scale N-T points from 12V to 14V
        self.scale_factor_tor = self.max_tor_14V / self.max_tor_12V
        self.scale_factor_vel = self.max_vel_14V / self.max_vel_12V
        self.NT_curve_point_1_14V = Point(
            self.NT_curve_point_1_12V.x * self.scale_factor_tor, self.NT_curve_point_1_12V.y * self.scale_factor_vel)
        self.NT_curve_point_2_14V = Point(
            self.NT_curve_point_2_12V.x * self.scale_factor_tor, self.NT_curve_point_2_12V.y * self.scale_factor_vel)

    def get_values(self, use_14V):
        # choose correct values based on voltage
        if use_14V:
            stall_torque = self.max_tor_14V
            vel = self.max_vel_14V
            point_1 = self.NT_curve_point_1_14V
            point_2 = self.NT_curve_point_2_14V
        else:
            stall_torque = self.max_tor_12V
            vel = self.max_vel_12V
            point_1 = self.NT_curve_point_1_12V
            point_2 = self.NT_curve_point_2_12V

        # compute line which intersects with both NT points
        # y=ax+b
        # point_1.y = a*point_1.x+b
        # point_2.y = a*point_2.x+b
        # b = point_1.y - a*point_1.x
        # point_2.y = a*point_2.x + point_1.y - a*point_1.x
        # point_2.y - point_1.y = a*(point_2.x - point_1.x)
        a = (point_2.y - point_1.y) / (point_2.x - point_1.x)
        b = point_1.y - (point_1.x * a)

        # compute torque at vel=0
        # 0 = a*x+b -> x = -b / a
        torque_vel0 = -b / a
        friction = stall_torque - torque_vel0

        print(f"{self.name}-torque    {stall_torque}")
        print(f"{self.name}-vel    {vel}")  # todo this could also be "b"
        print(f"{self.name}-damping    {-a}")  # damping is already interpreted negatively
        print(f"{self.name}-friction    {friction}")
        print("")


# max_tor_12V, max_tor_14V, max_vel_14V_rpm, max_vel_12V_rpm, NT_curve_point_1, NT_curve_point_2
V14 = True
joint_specs("MX64", 6.0, 7.3, 63, 78, Point(0.15, 64), Point(2.85, 25)).get_values(V14)
joint_specs("MX106", 8.4, 10, 45, 55, Point(0.7, 42), Point(5.6, 5)).get_values(V14)
joint_specs("XH540W270", 10.6, 12.9, 30, 37, Point(0.4, 29), Point(8.6, 2.5)).get_values(V14)
