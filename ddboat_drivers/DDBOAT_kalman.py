#!/usr/bin/env python3
################################################
# Auteurs : Morgan Louedec & BATOTOCLOCLOJOJO & tarpon
# Date : 25/02/2022
# Libre de droit mais citer les auteurs
################################################
# This file is a library to manage the measurements of the DDBOATs
#################################################
import numpy as np
from math import cos, sin, pi

class StateObserver:
    # Extended Kalman filter to estimate position and speed
    def __init__(self, X0: np.array, th0: float, Gamma0: np.array, Gamma_alpha: np.array, Gamma_beta: np.array,
                 dt: float):
        self.X = X0  # initial state (x,y,v,qx,qy)
        self.Gamma = Gamma0
        self.Gamma_alpha = Gamma_alpha
        self.Gamma_beta = Gamma_beta
        self.dt = dt  # loop period
        self.u = np.array([[0, 0]]).T
        self.y = np.array([[0, 0]]).T
        self.th = th0  # measurement of the heading
        self.Ck = np.array([[1, 0, 0, 0,0], [0, 1, 0, 0,0]])
        print("kalman filter created")

    def p(self):  # robot position
        return self.X[0:2, :]

    def p_dot(self):  # robot speed vector
        return np.array([[self.X[2, 0] * cos(self.th) + self.X[3, 0], self.X[2, 0] * sin(self.th) + self.X[4, 0]]]).T

    def Kalman_update(self, u,th):  # time update based on the movement of the robot
        # u : [acceleration, angular velocity] control signal
        self.u,self.th = u,th
        Ak = np.eye(5) + self.dt * np.array(
            [[0, 0, cos(self.th), 1, 0],
             [0, 0, sin(self.th), 0, 1],
             [0, 0, 0, 0, 0],
             [0, 0, 0, 0,0],
             [0, 0, 0, 0, 0]])
        self.X = self.X + self.dt * np.array([[self.X[2,0] * cos(self.th)+self.X[3,0]],
                                              [self.X[2,0] * sin(self.th)+self.X[4,0]],
                                              [u[0, 0]],
                                              [0],
                                              [0]])
        self.Gamma = Ak @ self.Gamma @ Ak.T + self.Gamma_alpha
        return

    def Kalman_correct(self, y):  # measurement correction of the kalman filter
        # y : measure [x,y]
        self.y = y
        Zk = y - self.Ck @ self.X # g(x) is linear
        Sk = self.Ck @ self.Gamma @ self.Ck.T + self.Gamma_beta
        Kk = self.Gamma @ self.Ck.T @ np.linalg.inv(Sk)
        self.X = self.X + Kk @ Zk
        self.Gamma = (np.eye(5) - Kk @ self.Ck) @ self.Gamma
        return
