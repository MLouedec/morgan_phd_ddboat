# display heading
import numpy as np

from DDBOAT_controller import *
from DDBOAT_kalman import *
from DDBOAT_log import LogRecorder, init_drivers, time, robot_number
import json
import sys

filename = "compass_calibration/compass_calibration_ddboat"+robot_number+".json"

file_script = open("mission_script_old.json", "r")
file_script2 = open(filename, "r")
data_script = json.load(file_script)
data_script2 = json.load(file_script2)
param = data_script["mission_param"]

lxm = param["lxm"]
lym = param["lym"]
b = np.reshape(np.array([data_script2["b"]]), (3, 1))
A = np.reshape(np.array([data_script2["A"]]), (3, 3))

ard, temperature, gps, encoddrv, imu = init_drivers()
filt = DdboatFilter(lxm, lym, A, b, encoddrv)
while True:

    # measurements
    mag = imu.read_mag_raw()
    y_th = 360/(2*3.14)*filt.cap(mag[0], mag[1], mag[2])
    print("campass value",y_th)
    time.sleep(0.01)
