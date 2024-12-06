# control function from DDBOAT_controller.py adapted for the simulation
from sympy.physics.vector import cross

from boat import sawtooth
import numpy as np
from math import atan2

k_heading = 0.7 # heading control gain
k_speed = 0.7 # speed control gain
ki_speed = 0.1 # integral gain for the speed control
kd_speed = 2*k_speed # derivative gain for the speed control

def heading_regul(speed_d, th_d, th):  # compute the motor control signal
    # speed_d: desired speed (m/s)
    # th_d: desired heading (rad)
    # th: current heading (rad)
    w = k_heading * sawtooth(th_d - th)  # desired angular speed
    vd = np.array([speed_d, w])
    return vd

# def follow_point(b, m):
#     # compute the desired heading to follow the point b in straight line
#     # b: target position
#     # m: current position
#     return np.arctan2(b[1] - m[1], b[0] - m[0])

# def follow_point_v2(b, m):
#     # compute the desired heading to follow the point b in straight line
#     # reduce the speed when approaching the target
#     # b: target position
#     # m: current position
#     thd = np.arctan2(b[1] - m[1], b[0] - m[0])
#     dist_error = np.linalg.norm(b - m)
#     speed_d = k_speed * dist_error
#     return thd, speed_d
#
# def follow_point_v3(b, m,th, dt, integral = 0):
#     # same as v2 but add a integral term in the speed control
#     # b: target position
#     # m: current position
#     thd = np.arctan2(b[1] - m[1], b[0] - m[0])
#     dist_error = np.linalg.norm(b - m)
#     front = np.dot(np.array([np.cos(th), np.sin(th)]), b - m)
#     if front > 0:
#         integral += dt*front
#     else:
#         integral = 0.*integral
#     speed_d = k_speed * dist_error + ki_speed * integral
#     return thd, speed_d, integral

def follow_pose(pd,pose,dt,v1_old=0,pd_dot=np.array([0,0]),pd_ddot=np.array([0,0])):
    # feedback linearization to follow a pose
    # pd : desired position [xd,yd]
    # pose : current pose [x,y,theta]
    # dt : time step
    # v1_old : previous speed
    # return : desired speed

    #optional parameters
    # pd_dot : desired speed [xd_dot,yd_dot]
    # pd_ddot : desired acceleration [xd_ddot,yd_ddot]

    ex = k_speed * (pd[0]-pose[0])+ kd_speed * (pd_dot[0]-v1_old*np.cos(pose[2])) + pd_ddot[0]
    ey = k_speed * (pd[1]-pose[1])+ kd_speed * (pd_dot[1]-v1_old*np.sin(pose[2])) + pd_ddot[1]

    v1 = v1_old + dt * (np.cos(pose[2]) * ex + np.sin(pose[2]) * ey)

    if abs(v1) < 0.01:
        # avoid singularity
        v2 = 0
    else:
        v2 = -np.sin(pose[2])/v1 * ex + np.cos(pose[2])/v1 * ey

    if v1<0:
        # the robot can move backward, turn left
        v1 = 0.
        v2 = 100
    return np.array([v1,v2])

# def clever_follow_point(b, m, th, radius=2, speed_max=2):
#     # compute the desired heading and the desired speed to follow the point b
#     # assuming that the robot can't move foward and that the target is moving
#     # the robot can stop to wait for the target to come
#     # the robot also reduces its speed when approaching the target
#     # but it keep a minimal speed equal to the target speed
#
#     # b: target position
#     # m: current position
#     # max_v: max speed of the robot
#     # radius: radius under which the robot stops
#     # integral: saturated integral of the error
#
#     thd = np.arctan2(b[1] - m[1], b[0] - m[0]) # heading towards the target
#     dist_error = np.linalg.norm(b - m)
#     vect = np.array([np.cos(th), np.sin(th)])
#
#     # case 1, the target is in front of the robot
#     front = np.dot(vect, b - m)
#     if front>0:
#         speed_d= min(speed_max * (dist_error / radius)**2, speed_max)
#     else:
#         # case 2, the target is behind the robot under the radius
#         if dist_error < radius:
#             speed_d = 0 # don't move
#         else:
#             # the robot must make a full turn
#             speed_d = speed_max
#     return thd, speed_d


def follow_line(a, b, m):
    # compute the desired heading to follow the line [ab)
    # a: beginning of the line
    # b: end of the line
    # m: current position
    r = 4  # gain
    u = (b - a) / np.linalg.norm(b - a)  # unit vector of the line
    e = np.linalg.det(np.vstack([u, m - a]))  # distance error with the line
    phi = np.arctan2(b[1] - a[1], b[0] - a[0])  # orientation of the line
    # print("phi",phi)
    # print("correct",np.arctan(e/r))
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
    pd = c + R * np.array([np.cos(phi1),
                           np.sin(phi2)])
    pd_dot = R * np.array([-np.pi * 2 / T * np.sin(phi1),
                           np.pi * 4 / T * np.cos(phi2)])
    pd_ddot = R * np.array([-(np.pi * 2 / T) ** 2 * np.cos(phi1),
                            -(np.pi * 4 / T) ** 2 * np.sin(phi2)])
    return pd, pd_dot, pd_ddot

#
# def control_station_keeping(a, d, pos, th, speed_d):
#     # controller to keep the robot next ot a position
#     # a: reference position (local position, m)
#     # d: max distance desired (m)
#     # pos : current position of the robot (local position, m)
#     # th:  current heading (rad)
#     # v: desired speed (m/s)
#
#     d_mes = np.linalg.norm(pos - a)
#     if d_mes > d:
#         th_d = follow_point(a, pos)
#         return heading_regul(speed_d, th_d, th)
#     else:
#         # set speed to 0
#         return np.zeros(2)
    
# class StationKeeping:
#     # automat for station keeping
#     def __init__(self):
#         self.state = 0  # state of the automat
#         self.t_burst = 0
#         self.e = 0
#         self.pd1 = np.zeros((2, 1))
#
#     def burst(self, t,t1=10,t2=5):
#         #t1 : time between the bursts (sec)
#         #t2 : duration of a bust (sec)
#         Delta_t = t - self.t_burst
#         if t1 + t2 > Delta_t > t1:
#             return np.array([1,0])
#         elif Delta_t > t1 + t2:
#             self.t_burst = t
#         return np.zeros(2)
#
#     def station_keeping1(self,pd0,p,th,speed_d,t,r=4):
#         # basic station keeping with in and out circle
#         # pd0 : desired position
#         # p : current position
#         # th : current heading
#         # speed_d : desired speed
#         # t : current time
#         # r : radius of the station keeping circle
#
#         # change state
#         if self.state == 0 and np.linalg.norm(pd0 - p) < r / 2:
#             self.state = 1
#             self.t_burst = t
#         if self.state == 1 and np.linalg.norm(pd0 - p) > r:
#             self.state = 0
#
#         # control
#         if self.state == 0:
#             th_d = follow_point(pd0,p)
#             vd = heading_regul(speed_d, th_d, th)
#             return vd
#         if self.state == 1:
#             return self.burst(t)
#
#     def station_keeping2(self,pd0,pd_dot0,p,th,speed_d,t,r=4):
#         # advanced station keeping to have correct heading
#         # pd0 : desired position
#         # pd_dot0 : desired speed at the desired position
#         # p : current position
#         # th : current heading
#         # speed_d : desired speed
#         # t : current time
#         # r : radius of the station keeping circle
#
#         # change state
#         if self.state == 0:
#             self.pd1 = pd0 - 0.5 * r * pd_dot0 / np.linalg.norm(pd_dot0)  # desired position for state 0
#             if np.linalg.norm(self.pd1 - p) < r / 2:
#                 self.state = 1
#         if self.state == 1:
#             th_d0 = atan2(pd_dot0[1], pd_dot0[0])
#             self.e = sawtooth(th_d0 - th) # angle error
#             if abs(self.e) < 0.1:
#                 self.state = 2
#                 self.t_burst = t
#         if self.state == 2 and np.linalg.norm(pd0 - p) > r:
#             self.state = 0
#
#         # compute control
#         if self.state == 0:  # go towards pd1
#             th_d = follow_point(self.pd1, p)
#             return heading_regul(speed_d, th_d, th)
#         if self.state == 1:  # turn towards th_d heading
#             w_d =  0.1 * self.e
#             return np.array([speed_d,w_d])
#         if self.state == 2:  # don't move
#             return self.burst(t)