#!/usr/bin/env python3
################################################
# Auteurs : Morgan Louedec & BATOTOCLOCLOJOJO & tarpon
# Date : 23/02/2022
# Libre de droit mais citer les auteurs
################################################
# This file is a library for the controllers of the DDBoat
# current controllers:
# speed and heading / regul(v_d, th_d, th)
# point following / follow_point(b,m)
# line following / follow_line(a, b, m)
# station_keeping / control_station_keeping(a, d, pos, th, v)
# Lissajou curve / control_lisssajou(...)

# these function can be used as a controler bloc
# inputs : local positions (m), heading(rad), time(s), references
# output : left and right thruster speed (m/s)
#################################################

import numpy as np
from math import sin, cos, atan2, sqrt

B = np.array([[1, 1], [-1, 1]])

kT, kpwm, kD, kw = 0.01, 1., 1, 0.06  # controller gains
m = 2.5  # weight of the DDBOAT (kg) (please update)
umax = 200 # max pwm allowed

kp, kd, kth = 1, 2, 1
K_inv = 1 / (2 * kT) * np.array([[m, -1 / kw], [m, 1 / kw]]) # [wl*wl wr*wr].T = K_inv * ([v_dot, w*|w|].T + D/m)
B = kT*np.array([[1/kD,1/kD],[-kw,kw]]) # [v*v,w*|w|] = B * [wl*wl wr*wr].T
B_inv = np.linalg.inv(B)#

k_heading = 0.5  # heading controller gain (s^{-1}) /  high gain = 5

def sawtooth(x):
    return (x + np.pi) % (2 * np.pi) - np.pi

def convert_motor_control_signal(vd,wd): # convert desired speed to motor control signal (open loop)
    # vd desired speed (m/s)
    # wd desired angular speed (rad/s)
    wm_sqr = B_inv @ (np.array([[vd*vd], [wd * abs(wd)]]))  # [wl**2 , wr**2]

    # square must not be negative
    wm_sqr[0,0] = max(0,wm_sqr[0,0])
    wm_sqr[1,0] = max(0,wm_sqr[1,0])

    wmLeft_d, wmRight_d = sqrt(wm_sqr[0, 0]), sqrt(wm_sqr[1, 0])

    # convert to pwm
    cmdL = min(max(kpwm * wmLeft_d, 0), umax)
    cmdR = min(max(kpwm * wmRight_d, 0), umax)

    return cmdL, cmdR  # controlled PWM

def heading_regul(v_d, th_d, th):  # compute the motor control signal
    # v_d: desired speed (m/s)
    # th_d: desired heading (rad)
    # th: current heading (rad)
    w = k_heading * sawtooth(th_d - th)  # desired angular speed
    cmdL,cmdR =  convert_motor_control_signal(v_d,w)
    return cmdL,cmdR,w

def follow_point(b, m):
    # compute the desired heading to follow the point b in straight line
    # b: target position
    # m: current position
    return np.arctan2(b[1, 0] - m[1, 0], b[0, 0] - m[0, 0])

def follow_line(a, b, m):
    # compute the desired heading to follow the line [ab)
    # a: beginning of the line
    # b: end of the line
    # m: current position
    r = 4  # gain
    u = (b - a) / np.linalg.norm(b - a)  # unit vector of the line
    e = np.linalg.det(np.hstack([u, m - a]))  # distance error with the line
    phi = np.arctan2(b[1, 0] - a[1, 0], b[0, 0] - a[0, 0])  # orientation of the line
    print("phi",phi)
    print("correct",np.arctan(e/r))
    ang = sawtooth(phi - np.arctan(e / r))  # desired heading
    # print("desired ANG (deg)", ang * 180 / np.pi)
    return ang

def lissajou_trajectory(c, R, T, t, i, N):
    # compute the Lissajou trajectory and its derivatives
    # c : center of the curve
    # R : amplitude of the curve
    # T : period of the trajectory
    # t : current time since the beginning of the mission
    # i : id of the robot
    # N : total number of robots
    phi1, phi2 = t * np.pi * 2 / T + 2 * i * np.pi / N, 2 * (t * np.pi * 2 / T + 2 * i * np.pi / N)
    pd = c + R * np.array([[np.cos(phi1)],
                           [np.sin(phi2)]])
    pd_dot = R * np.array([[-np.pi * 2 / T * np.sin(phi1)],
                           [np.pi * 4 / T * np.cos(2 * phi2)]])
    pd_ddot = R * np.array([[-(np.pi * 2 / T) ** 2 * np.cos(phi1)],
                            [-(np.pi * 4 / T) ** 2 * np.sin(phi2)]])
    return pd, pd_dot, pd_ddot


def control_lisssajou(p, th, pd, min_v=0.1, max_v=10):
    # compute the control signal (u1,u2) to follow a Lissajou curve with only distance error
    # p: current position (local coord)
    # th: current heading (rad)
    # pd: desired position (m) [2D vector]
    # min_v: min speed (m/s)
    # max_v: max speed (m/s)

    th_d = follow_point(pd, p)  # follow the line [mX)
    dist_error = np.linalg.norm(pd - p)
    v = min((1 + 0.1 * dist_error ** 2) * min_v, max_v)  # speed depends on quadratic error and is saturated
    return heading_regul(v, th_d, th)


def control_station_keeping(a, d, pos, th, v=1):  # controller to keep the robot next ot a position
    # a: reference position (local position, m)
    # d: max distance desired (m)
    # pos : current position of the robot (local position, m)
    # th:  current heading (rad)
    # v: desired speed (m/s)

    d_mes = np.linalg.norm(pos - a)
    if d_mes > d:
        th_d = follow_point(a, pos)
        return heading_regul(v, th_d, th)
    else:
        return 0, 0

class StationKeeping:
    # automat for station keeping
    def __init__(self):
        self.state = 0  # state of the automat
        self.t_burst = 0
        self.e = 0
        self.pd1 = np.zeros((2, 1))

    def burst(self, t,t1=10,t2=5):
        #t1 : time between the bursts (sec)
        #t2 : duration of a bust (sec)
        Delta_t = t - self.t_burst
        if t1 + t2 > Delta_t > t1:
            return 75, 75
        elif Delta_t > t1 + t2:
            self.t_burst = t
        return 0, 0

    def station_keeping1(self,pd0,p,th,v_d,t,r=4):
        # basic station keeping with in and out circle
        # pd0 : desired position
        # p : current position
        # th : current heading
        # v_d : desired speed
        # t : current time
        # r : radius of the station keeping circle

        # change state
        if self.state == 0 and np.linalg.norm(pd0 - p) < r / 2:
            self.state = 1
            self.t_burst = t
        if self.state == 1 and np.linalg.norm(pd0 - p) > r:
            self.state = 0

        # control
        if self.state == 0:
            th_d = follow_point(pd0,p)
            cmdL,cmdR,_ = heading_regul(v_d, th_d, th)
            return cmdL,cmdR
        if self.state == 1:
            return self.burst(t)

    def station_keeping2(self,pd0,pd_dot0,p,th,v_d,t,r=4):
        # advanced station keeping to have correct heading
        # pd0 : desired position
        # pd_dot0 : desired speed at the desired position
        # p : current position
        # th : current heading
        # v_d : desired speed
        # t : current time
        # r : radius of the station keeping circle

        # change state
        if self.state == 0:
            self.pd1 = pd0 - 0.5 * r * pd_dot0 / np.linalg.norm(pd_dot0)  # desired position for state 0
            if np.linalg.norm(self.pd1 - p) < r / 2:
                self.state = 1
        if self.state == 1:
            th_d0 = atan2(pd_dot0[1, 0], pd_dot0[0, 0])
            self.e = sawtooth(th_d0 - th) # angle error
            if abs(self.e) < 0.1:
                self.state = 2
                self.t_burst = t
        if self.state == 2 and np.linalg.norm(pd0 - p) > r:
            self.state = 0
        print(self.state)

        # compute control
        if self.state == 0:  # go towards pd1
            th_d = follow_point(self.pd1, p)
            cmdL, cmdR, _ = heading_regul(v_d, th_d, th)
            return cmdL, cmdR
        if self.state == 1:  # turn towards th_d heading
            w_d =  0.1 * self.e
            cmdL, cmdR = convert_motor_control_signal(v_d,w_d)
            return cmdL, cmdR
        if self.state == 2:  # don't move
            return self.burst(t)

# -------------------------------
# EXPERIMENTAL CONTROL
# -------------------------------
# wlrmax = umax / kpwm # max motor rotation speed
# vmax = sqrt(2 * kT / kD) * wlrmax # max speed allowed
# wmax = sqrt(kw * kT) * wlrmax # max angular speed allowed
# rmax = vmax / wmax # max turning radius
# amax = 2 * kT * umax ** 2 / (m * kpwm ** 2)
#
# def control_feedback_linearization(pd, pd_dot, pd_ddot, dt, p, v, th, qx, qy):
#     # compute the control signal (u1,u2) to follow the desired trajectory
#     # (pd,pd_dot,pd_ddot) desired position, velocity, acceleration (in x , y frame)
#     # (p,p_dot) : measured position and velocity
#     # (qx,qy): estimated speed bias caused by the wind
#     # dt: controller period
#
#     if abs(v) < 0.1:  # avoid singularity
#         # ~ print("speed singularity in controller")
#         return np.array([[2, 0]]).T
#
#     e = p - pd  # position error
#     ed = np.array([[v * cos(th) + qx, v * sin(th) + qy]]).T - pd_dot  # velocity error
#     A = np.array([[cos(th), -v * sin(th)], [sin(th), v * cos(th)]])  # transition matrix, p_ddot = A*u
#     kc = 0.1
#     u = np.linalg.inv(A) @ (pd_ddot - kd * ed - kp * e)  # [scalar acceleration, angular velocity]
#     u = control_saturate(u, v)
#     return u
#
# def control_feedback_linearization2(pd, pd_dot, pd_ddot, dt, p, v, th, qx, qy, r=4):
#     # as above but modified. the robot wait if ahead of the reference bellow r meters of distance
#
#     if np.linalg.norm(pd - p) < r and np.dot(np.transpose(p - pd), pd_dot) > 0:
#         th_d = atan2(pd_dot[1, 0], pd_dot[0, 0])
#         e = sawtooth(th_d - th)
#         u = np.array([[-0.1 * np.linalg.norm(pd_dot)], [0.1 * e]])
#         u = control_saturate(u, v)
#     else:
#         u = control_feedback_linearization(pd, pd_dot, pd_ddot, dt, p, v, th, qx, qy)
#
#     return u
#
# def control_saturate(u, v): # saturation of u = [v_dot,w] to avoid motor saturation
#     # the acceleration is constrained by the angular velocity
#     u[1,0] = min(wmax,max(-wmax,u[1,0])) # -wmax < w < wmax
#     D = kd * v * abs(v) # damping force
#     al = -D / m + u[1, 0] ** 2 / (kw * m) # minimun acceleration possible
#     aL = amax - D / m - u[1, 0] ** 2 / (kw * m) # maximum acceleration possible
#     u[0, 0] = min(max(al, u[0, 0]), aL) # al < a < aL
#     return u
#
# class ControlBlock:
#     def __init__(self, dt, traj0, r=4):
#         self.state = 0  # 0 : move towards waypoint , 1 : move towards desired heading, 2 : stop
#         self.dt = dt
#         try:
#             self.pd0 = np.reshape(np.array([traj0["pd"]]), (2, 1))
#             self.pd1 = self.pd0
#             self.pd_dot0 = np.reshape(np.array([traj0["pd_dot"]]), (2, 1))
#             self.th_d0 = atan2(self.pd_dot0[1, 0], self.pd_dot0[0, 0])
#             self.pd_ddot0 = np.reshape(np.array([traj0["pd_ddot"]]), (2, 1))
#         except:
#             print("no trajectory loaded")
#             self.pd0 = np.zeros((2,1))
#             self.pd1 = self.pd0
#             self.pd_dot0 = self.pd0
#             self.th_d0 = self.pd0
#             self.pd_ddot0 = self.pd0
#
#         self.r = r  # radius of the station keeping circle
#         self.t_burst = 0.0  # burst time for state 1
#
#     def variable_update(self, p, v, th, qx, qy, wmLeft, wmRight, cmdL_old, cmdR_old):
#         self.p = p
#         self.v = v
#         self.th = th
#         self.qx = qx
#         self.qy = qy
#         self.wmLeft = wmLeft
#         self.wmRight = wmRight
#         self.cmdL_old = cmdL_old
#         self.cmdR_old = cmdR_old
#         return
#
#     def follow_reference(self, pd, pd_dot,
#                          pd_ddot):  # make control_feedback_linearization and convert_motor_control_signal
#         u = control_feedback_linearization(pd, pd_dot, pd_ddot, self.dt, self.p, self.v, self.th, self.qx, self.qy)
#         cmdL, cmdR = convert_motor_control_signal(u, self.v, self.wmLeft, self.wmRight, self.cmdL_old, self.cmdR_old,
#                                                   self.dt)
#         return cmdL, cmdR, u
#
#     def follow_reference2(self, pd, pd_dot=np.zeros((2, 1)), pd_ddot=np.zeros((2, 1)),
#                           r=4):  # same as above with control_feedback_linearization2
#         u = control_feedback_linearization2(pd, pd_dot, pd_ddot, self.dt, self.p, self.v, self.th, self.qx, self.qy, r)
#         cmdL, cmdR = convert_motor_control_signal(u, self.v, self.wmLeft, self.wmRight, self.cmdL_old, self.cmdR_old,
#                                                   self.dt)
#         return cmdL, cmdR, u
#
#     def burst(self, t):
#         Delta_t = t - self.t_burst
#         # ~ print(Delta_t)
#         t1, t2 = 30, 5
#         if t1 + t2 > Delta_t > t1:  # bust of 1 sec every 10 sec
#             return 75, 75, np.zeros((2, 1))
#         elif Delta_t > t1 + t2:
#             self.t_burst = t
#         return 0, 0, np.zeros((2, 1))
#
#     def station_keeping1(self, t):  # basic station keeping with in and out circle
#         # change state
#         if self.state == 0 and np.linalg.norm(self.pd0 - self.p) < self.r / 2:
#             self.state = 1
#             self.t_burst = t
#         if self.state == 1 and np.linalg.norm(self.pd0 - self.p) > self.r:
#             self.state = 0
#         # ~ print(self.state)
#         # ~ print(np.linalg.norm(self.pd0 - self.p))
#
#         # control
#         if self.state == 0:
#             return self.follow_reference(self.pd0, self.pd_dot0, self.pd_ddot0)
#         if self.state == 1:
#             return self.burst(t)
#
#     def station_keeping2(self, t):
#         # advanced station keeping to have correct heading
#
#         # change state
#         if self.state == 0:
#             self.pd1 = self.pd0 - 0.5 * self.r * self.pd_dot0 / np.linalg.norm(
#                 self.pd_dot0)  # desired position for state 0
#             if np.linalg.norm(self.pd1 - self.p) < self.r / 2:
#                 self.state = 1
#         if self.state == 1:
#             e = sawtooth(self.th_d0 - self.th)
#             if abs(e) < 0.1:
#                 self.state = 2
#                 self.t_burst = t
#         if self.state == 2 and np.linalg.norm(self.pd0 - self.p) > self.r:
#             self.state = 0
#         print(self.state)
#
#         # compute control
#         if self.state == 0:  # go towards pd1
#             return self.follow_reference(self.pd1, self.pd_dot0, self.pd_ddot0)
#         if self.state == 1:  # turn towards th_d heading
#             u = np.array([[0, 0.1 * e]]).T
#             cmdL, cmdR = convert_motor_control_signal(u, self.v, self.wmLeft, self.wmRight, self.cmdL_old,
#                                                       self.cmdR_old, self.dt)
#             return cmdL, cmdR, u
#         if self.state == 2:  # don't move
#             return self.burst(t)