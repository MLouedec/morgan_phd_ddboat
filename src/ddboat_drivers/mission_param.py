#!/usr/bin/env python3
from DDBOAT_controller import *
from DDBOAT_kalman import *
from log_driver import LogRecorder, init_drivers, time, robot_number
import json
import sys


class MissionBlock:
    def __init__(self, rh):  # mission initialisation
        print("robot setup ...")

        # load mission script
        file_param = open("mission_script.json", "r")
        file_script2 = open("compass_calibration/compass_parameters_" + robot_number + ".json", "r")
        param = json.load(file_param)
        data_script2 = json.load(file_script2)
        # trajs = data_script["trajectories"]
        robot_id = param["robot_id"]
        print("robot id", robot_id)

        # deal with starting time of the mission
        try:
            start_time = param["start_time"]
            year = start_time["year"]
            month = start_time["month"]
            day = start_time["day"]
            hour = start_time["hour"]
            minute = start_time["minute"]
            dtStr = str(year) + "-" + str(month) + "-" + str(day) + "-" + str(hour) + "-" + str(minute) + "-" + "0"
            local_time_mission_begin = time.strptime(dtStr, "%Y-%m-%d-%H-%M-%S")
            self.time_mission_begin = time.mktime(local_time_mission_begin)
        except:
            print("the mission start right now !")

        dt = param["dt"]  # loop time period
        self.time_mission_max = param["duration_mission_max"] + time.time()  # max allowed time for mission

        # drivers
        self.ard, self.temperature, self.gps, self.encoddrv, self.imu = init_drivers()

        self.imu.setup_accel_filter(2)
        self.imu.load_calibration_parameters("compass/compass_parameters_+"+robot_id+".json")

        # coordinates of the origin
        self.lxm = param["lxm"]
        self.lym = param["lym"]

        # Log
        self.log_rec = LogRecorder(local_time_mission_begin)

        # find initial pose
        X0, y_th = self.wait_for_position_measurement()

        # kalman filter
        Gamma0 = np.diag(param["Gamma0"])
        Gamma_alpha = np.diag(param["Gamma_alpha"])
        Gamma_beta = np.diag(param["Gamma_beta"])
        self.kal = StateObserver(X0, y_th, Gamma0, Gamma_alpha, Gamma_beta, self.dt)

        # set home
        self.return_home = rh  # if true, at the en of the mission, the robot return home
        home_lat, home_lon = param["home_lat"], param["home_lon"]
        self.home_pos = self.filt.latlon_to_coord(home_lat, home_lon)

        # control
        self.CB = ControlBlock(self.dt, self.traj[0], r=4)

        # init variables
        self.t, self.sync, self.y_th = 0, False, 0

        print("robot setup done")
        print("---")
        print("mission will begin at", time.asctime(local_time_mission_begin))
        print("Going to the initial waypoint")

    def wait_for_position_measurement(self):
        while True:
            _, _, _, gll_ok, val, _, _, mag, _, _ = self.log_rec.log_observe_update(self.temperature, self.ard,
                                                                                    self.gps, self.encoddrv, self.imu)
            self.log_rec.log_update_write()
            if gll_ok:
                y_th = self.filt.cap(mag[0], mag[1], mag[2])
                lat, lon = self.filt.cvt_gll_ddmm_2_dd(val)
                pos = self.filt.latlon_to_coord(lat, lon)
                X0 = np.array([[pos[0, 0], pos[1, 0], 1, 0, 0]]).T  # note: initial speed set to 1 to avoid singularity
                break
            time.sleep(0.1)
        return X0, y_th

    def measure(self, cmdL_old, cmdR_old):
        # measurements
        self.t = time.time()
        _, _, _, gll_ok, val, self.sync, data_encoders, mag, _, _ = self.log_rec.log_observe_update(self.temperature,
                                                                                                    self.ard,
                                                                                                    self.gps,
                                                                                                    self.encoddrv,
                                                                                                    self.imu)
        self.wmLeft, self.wmRight = self.filt.measure_wm(data_encoders, self.dt)
        self.y_th = self.filt.cap(mag[0], mag[1], mag[2])
        if gll_ok:  # correct kalman filter with new gps data
            lat, lon = self.filt.cvt_gll_ddmm_2_dd(val)
            pos = self.filt.latlon_to_coord(lat, lon)
            self.kal.Kalman_correct(np.array([[pos[0, 0], pos[1, 0]]]).T)
        self.CB.variable_update(p=self.kal.p(), v=self.kal.X[2, 0], th=self.y_th, qx=self.kal.X[3, 0],
                                qy=self.kal.X[4, 0],
                                wmLeft=self.wmLeft, wmRight=self.wmRight, cmdL_old=cmdL_old, cmdR_old=cmdR_old)


