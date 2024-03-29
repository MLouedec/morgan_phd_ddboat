# DDBOAT python3 drivers version 2 - fork modified by morgan

The drivers are :
* IMU (MAG, ACCEL, GYRO) : imu9_driver_v2.py
* GPS (serial line, GPGLL message) : gps_driver_v2.py
* Encoders on propeller rotation (serial line) : encoders_driver_v2.py
* Arduino motors command (serial line) : arduino_driver_v2.py
* TC74 temperature sensors (one per motor) : tc74_driver_v2.py

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
