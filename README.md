# Custom version of the DDBOAT drivers
original version : https://gitlab.ensta-bretagne.fr/zerrbe/drivers-ddboat-v2

The drivers are :
* IMU (MAG, ACCEL, GYRO) : imu9_driver_v2.py
* GPS (serial line, GPGLL message) : gps_driver_v2.py
* Arduino motors command (serial line) : arduino_driver_v2.py

The controller loop is composed of:
* filter : DDBOAT_filter_v1.py
* low level control : DDBOAT_controler.py
* mission block toolbox : mission_param.py
* trajectory tracking navigation : Mission_trajectory_tracking.py | Log saved in Log directory

Mission instruction descibed by : mission_script.json

Additional tests:
* synchronise time with GPS (hour min sec) : gps_driver_v3.py
* synchronise time manually (year month day) : set_date.py
* test motor controller : test_motor_control_loop.py
* test the heading : test_compass.py
* display heading and local global position : scout.py

Compass parameters in compass_calibration/

# the compass is calibrated using the tools from https://github.com/godardma/magnetic_calibration

compass_recorder.py
creates the data.txt file with a record of the magnetic measurement x y z

after the calibration, the compass_parameters.json file is update
in this file there are two parameters b (3d vector) and A (3x3 matrix)

the correction is made with the linaer transformation
mag_corrected = A * (mag - b)

# the client_scripts folder contains the scripts to run on ground pc to communicate with the boat
