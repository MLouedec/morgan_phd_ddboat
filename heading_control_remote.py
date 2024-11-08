# the robot is controlled to reach the desired heading
# the desired heading is send by a remote computer with a client/service tcp socket

import time
from DDBOAT_controler_v1 import *
from imu9_driver_v2 import Imu9IO
from arduino_driver_v2 import ArduinoIO
from tcp_server_v2 import *
import math

# initialization
imu = Imu9IO()
imu.load_calibration_parameters("compass/compass_parameters.json")

ard = ArduinoIO()
ard.send_arduino_cmd_motor (0,0)


#####################
# mission loop
#####################
print("remote heading control")

# init the server
DS = DdboatServer()

# Main thread can perform other tasks
try:
    for i in range(6000):
        raw_mag = imu.read_mag_raw()
        mag_cal = imu.cal_mag(raw_mag)
        pitch, roll = imu.get_pitch_roll()
        th = imu.heading_simple(mag_cal)  # rad
        DS.current_heading = th

        th_d = DS.desired_heading  # desired heading
        cmdL, cmdR, w_d = heading_regul(0, th_d, th)

        # safely staturate the command in [0,50]
        cmdL = max(min(cmdL, 30), 0)
        cmdR = max(min(cmdR, 30), 0)

        ard.send_arduino_cmd_motor(cmdL, cmdR)

        print("\ndesired heading:", th_d * 180.0 / math.pi)
        print("current heading:", th * 180.0 / math.pi)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Server shutting down.")
    shutdown_flag = True
finally:
    ard.send_arduino_cmd_motor(0, 0)
    DS.server_socket.close()  # Close the server socket to unblock accept()
    DS.accept_thread.join()  # Wait for the accept thread to finish
    print("Server shut down.")








