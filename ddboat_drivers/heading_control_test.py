# the robot is controlled to reach the desired heading

import time
from DDBOAT_controller import *
from imu9_driver_v2 import Imu9IO
from arduino_driver_v2 import ArduinoIO
import math

# initialization
imu = Imu9IO()
imu.load_calibration_parameters("compass/compass_parameters_5.json")

ard = ArduinoIO()
ard.send_arduino_cmd_motor (0,0)


#####################
# mission loop
#####################
print("heading control")
th_d = 0 # desired heading


for i in range(6000):
    raw_mag = imu.read_mag_raw()
    # print("\nraw_data:", raw_mag, imu.read_accel_raw(), imu.read_gyro_raw())
    mag_cal = imu.cal_mag(raw_mag)
    pitch,roll = imu.get_pitch_roll()
    th = imu.heading_simple(mag_cal) # rad

    cmdL, cmdR, w_d = heading_regul(0,th_d,th)

    # safely staturate the command in [0,50]
    cmdL = max(min(cmdL,30),0)
    cmdR = max(min(cmdR,30),0)

    ard.send_arduino_cmd_motor(cmdL, cmdR)

    print("\ndesired heading:", th_d* 180.0 / math.pi)
    print("current heading:", th* 180.0 / math.pi)

    print("pitch:", pitch * 180.0 / math.pi)
    print("roll:", roll * 180.0 / math.pi)
    print("raw_accel:", imu.read_accel_raw())

    print("desired angular speed:",w_d)
    print("cmdL:",cmdL)
    print("cmdR:",cmdR)

    time.sleep(0.1)

ard.send_arduino_cmd_motor(0, 0)


