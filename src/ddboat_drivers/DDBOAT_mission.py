#!/usr/bin/env python3
import json
from DDBOAT_kalman import StateObserver
from DDBOAT_server import DdboatServer, memory
from gps_driver_v3 import latlon_to_coord
# from DDBOAT_log import cmdl_g, cmdr_g, roll_g, pitch_g, heading_g, pos_g
from DDBOAT_log import *
from DDBOAT_controller import *

SK = StationKeeping()

class MissionBlock:
    # the main class to manage the mission, it contains all the other classes
    def __init__(self,mission_filename):  # mission initialisation
        print("robot setup ...")
        print("loading ", mission_filename)

        # load mission script
        file_param = open(mission_filename, "r")
        print("file loaded")
        param = json.load(file_param)
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
            print("the mission will begin at", time.asctime(local_time_mission_begin))
        except:
            print("the mission start right now !")
            local_time_mission_begin = time.localtime()
            self.time_mission_begin = time.time()

        self.dt = param["dt"]  # loop time period
        self.time_mission_max = param["duration_mission_max"] + time.time()  # max allowed time for mission

        # drivers
        self.ard, self.gps, self.imu = init_drivers(2,robot_id=robot_id)

        # coordinates of the origin
        self.lxm = param["lxm"]
        self.lym = param["lym"]
        print("origin coordinates (lon,lat)", self.lxm, self.lym)

        # Log
        self.log_rec = LogRecorder(local_time_mission_begin,robot_id)

        # kalman
        try:
            Klm = param["Kalman"]
            Gamma0 = np.diag(Klm["Gamma0"])
            Gamma_alpha = np.diag(Klm["Gamma_alpha"])
            Gamma_beta = np.diag(Klm["Gamma_beta"])

            # find initial pose
            X0, y_th = self.wait_for_position_measurement()
            self.kal = StateObserver(X0, y_th, Gamma0, Gamma_alpha, Gamma_beta, self.dt)
        except:
            print("no kalman filter")
            self.kal = None

        with memory.lock:
            # control mode
            memory.control_mode = param["control_mode"]
            print("intial control mode", memory.control_mode)

        # home
        try:
            rh = param["return_home"]  # if true, at the en of the mission, the robot return home
            home_lat, home_lon = rh["home_lat"], rh["home_lon"]
            self.home_pos = latlon_to_coord(home_lat, home_lon,self.lxm, self.lym)
            print("home position", self.home_pos)
        except:
            print("no home")

        # server & memory
        serv = bool(param["server"])
        if serv:
            self.DS = DdboatServer()
            print("server connected")
        else:
            self.DS = None
            print("no server")


        # initial references
        with memory.lock:
            try:
                heading_d = param["heading_d"]
                print("initial desired heading (deg): ", heading_d)
                memory.heading_d  = float(heading_d) * 3.14159 / 180
            except:
                pass
            try:
                memory.pos_d = latlon_to_coord(param["lat_d"], param["lon_d"],self.lxm, self.lym)
                print("initial desired position (lat,lon): ", param["lat_d"], param["lon_d"])
                print("initial desired position (m): ", memory.pos_d )
            except:
                pass
            try:
                memory.speed_d  = param["speed_d"]
                print("initial desired speed (m/s): ", memory.speed_d )
            except:
                pass
            try:
                memory.rotation_speed_d  = param["rotation_speed_d"]
                print("initial rotation desired speed (rad/s): ", memory.rotation_speed_d )
            except:
                pass

        print("robot setup done")
        print("---")
        # print("mission will begin at", time.asctime(local_time_mission_begin))
        # print("Going to the initial waypoint")

    def wait_for_position_measurement(self):
        # wait for the first gll_ok gpt data to initialise the kalman filter
        T = 0.1 # loop period (sec)
        while True:
            t1 = time.time()
            _, _, gll_ok, lat,lon, _, _, heading, _, _, _ = self.log_rec.log_observe_update(self.ard,self.gps, self.imu)
            self.log_rec.log_update_write()
            if gll_ok:
                pos = latlon_to_coord(lat, lon,self.lxm, self.lym)

                # initialise the state of the kalman filter
                # note: initial speed set to 1 to avoid singularity
                X0 = np.array([[pos[0, 0], pos[1, 0], 1, 0, 0]]).T
                break

            t2 = time.time()
            if t2 - t1 < T:
                time.sleep(T - (t2 - t1))
        return X0, heading

    def wait_for_mission_begin(self):
        print("waiting for the mission to begin at time", time.asctime(time.localtime(self.time_mission_begin)))
        print("current time is", time.asctime(time.localtime(time.time())))
        print("time left:")
        while True:
            t = time.time()
            time_left = self.time_mission_begin - t
            print(time_left)
            if time_left < 0:
                break
            time.sleep(1)
        print("mission begin")

    def measure(self):
        # measurements
        (cmdl_g, cmdr_g, gll_ok, lat,lon,
         roll_g, pitch_g, heading_g,mag,accel,gyro) = self.log_rec.log_observe_update(self.ard,self.gps,self.imu)
        if gll_ok:
            pos_g = latlon_to_coord(lat, lon,self.lxm, self.lym)
            if self.kal: # correct kalman filter with new gps data
                self.kal.Kalman_correct(np.array([[pos_g[0, 0], pos_g[1, 0]]]).T)

        with memory.lock:
            memory.cmdl_g = cmdl_g
            memory.cmdr_g = cmdr_g
            memory.roll_g = roll_g
            memory.pitch_g = pitch_g
            memory.heading_g = heading_g
            if gll_ok:
                memory.pos_g = pos_g

    def mission_execution(self):
        init_time = time.time()
        while True:
            t1 = time.time()
            if t1 - init_time > self.time_mission_max:
                break
            self.measure()
            self.control()
            self.log_rec.log_update_write()

            t2 = time.time()
            if t2 - t1 < self.dt:
                time.sleep(self.dt - (t2 - t1))

    def control(self):
        with memory.lock:
            if memory.control_mode == "Standby":
                cmdL,cmdR = 0.,0.  # do nothing
            elif memory.control_mode == "Heading":
                # print("desired_heading", memory.heading_d)
                # print("current_heading", memory.heading_g)
                print("heading error", sawtooth(memory.heading_d - memory.heading_g))
                cmdL,cmdR,_ = heading_regul(memory.speed_d, memory.heading_d, memory.heading_g)
            elif memory.control_mode == "Waypoint":
                # 1 meter station keeping
                cmdL,cmdR = SK.station_keeping1(memory.pos_d,memory.pos,memory.heading_g,memory.speed_d,time.time(),r=1)
            elif memory.control_mode == "Speed":
                cmdL,cmdR = convert_motor_control_signal(memory.speed_d, memory.rotation_speed_d)
            else:
                print("control mode not implemented, I do nothing")
                cmdL,cmdR = 0.,0.  # do nothing
        self.ard.send_arduino_cmd_motor(cmdL, cmdR)



if __name__ == "__main__":
    # step 1: load the mission parameters
    try:
        mission_filename = sys.argv[1]
        MB = MissionBlock(mission_filename)
    except:
        print("mission initialisation failed")
        # print the error message
        print("Unexpected error:", sys.exc_info()[0])
        sys.exit()

    try:
        # step 2: wait for the beginning of the mission
        MB.wait_for_mission_begin()
        memory.current_goal()

        # step 3: execute the mission
        MB.mission_execution()

    except KeyboardInterrupt:
        print("Server shutting down.")
        shutdown_flag = True
    finally:
        MB.DS.server_socket.close() # Close the server socket to unblock accept()
        MB.DS.accept_thread.join() # Wait for the accept thread to finish
        print("Server shut down.")