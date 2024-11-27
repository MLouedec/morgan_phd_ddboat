# general test script

# step 1: check the drivers
import arduino_driver_v2 as ardu
import gps_driver_v3 as gpsdrv
import imu9_driver_v2 as imudv

ard = ardu.ArduinoIO()
gps = gpsdrv.GpsIO()
imu = imudv.Imu9IO()

print("IMU raw data:", imu.read_mag_raw(), imu.read_accel_raw(), imu.read_gyro_raw())
print("GPS raw data:", gps.read_gps())
print("Arduino raw data:", ard.get_arduino_status())

# step 2: test the log class
import DDBOAT_log as log

Log = log.LogRecorder(0,0)
for k in range(20):
    Log.log_observe_update(ard,gps,imu)
    Log.log_update_write()
print("log test done and recorded in Log/DDBOAT0_t_0.log") 
