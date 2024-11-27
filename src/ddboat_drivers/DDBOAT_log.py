import arduino_driver_v2 as ardu
import imu9_driver_v2 as imudv
import gps_driver_v3 as gpsdrv
import logging
import time
import sys

# noinspection PyBroadException
class LogRecorder:
    # create a log file and write in it
    def __init__(self,t=None,robot_id="Unknown"):
        # t : time in the name of the log file (optionnal, if None, current time is used)
        # robot_id : robot number (optionnal, if None, "Unknown" is used)

        # create a log file
        if t is None:
            t=time.localtime(time.time())
        number = str(t.tm_year) + "_" + str(t.tm_mon) + "_" + str(t.tm_mday) + "_" + str(t.tm_hour) + "_" + str(
            t.tm_min)
        logging.basicConfig(filename='Log/DDBOAT' + str(robot_id) + "_t_" + number + '.log', level=logging.DEBUG,
                            format='%(asctime)s - %(levelname)s - %(message)s')
        self.msg = ""  # string buffer for log messages
        print("log file created: Log/DDBOAT" + str(robot_id) + "_t_" + number + '.log')

    def log_observe_update(self, ard_, gps_, imu_):

        try:
            data = ard_.get_arduino_cmd_motor()
            data = data.split(",")
            cmdl = str(int(data[1].split(":")[1]))
            cmdr = str(int(data[3].split(":")[1]))

            self.msg = self.msg + "/cmdl:"+str(cmdl)+","
            self.msg = self.msg + "cmdr:"+str(cmdr)+"/"
        except:
            cmdl = None
            cmdr = None
            logging.error("can't log the motor command")

        try:
            gll_ok, val = gps_.read_gll_non_blocking()
            lat, lon = gpsdrv.cvt_gll_ddmm_2_dd(val)
            if gll_ok:
                self.msg = self.msg + "GPS " + str(val) + "/"
            else:
                self.msg = self.msg + "GPS-timeout/"
        except:
            gll_ok = False
            lat = None
            lon = None
            logging.error("can't log the GPS")

        try:
            mag = imu_.read_mag_raw()
            accel = imu_.read_accel_raw()
            gyro = imu_.read_gyro_raw()

            # show the calibrated
            mag_cal = imu_.cal_mag(mag)
            pitch, roll = imu_.get_pitch_roll(accel)
            heading = imu_.heading(mag,pitch,roll)
            self.msg = (self.msg + "IMU:MAG" + str(mag) + ",MAG_CAL" + str(mag_cal) +
                        ",ACCEL" + str(accel) + ",GYRO" + str(gyro) + "/" +
                        ",PITCH:" + str(pitch) + ",ROLL:" + str(roll) + ",YAW:" + str(heading) + "/")
        except:
            mag, accel, gyro = [], [], []
            roll, pitch, heading = None, None, None
            logging.error("can't log the imu")
        return cmdl, cmdr, gll_ok, lat,lon, roll, pitch, heading,mag,accel,gyro

    # def log_control_update(self, acc, wd, Wmleft, Wmright,CmdL,CmdR,pd, th,Kal=None):
    #     try:  # log desired pose
    #         self.msg = self.msg + "DESIREDPOSITION " + str(pd.tolist()) + " /"
    #     except:
    #         self.msg = self.msg
    #
    #     self.msg = self.msg + "CONTROL: ACCD " + str(acc) + " WD " + str(wd) + " "
    #     self.msg = self.msg + "WMLEFT " + str(Wmleft) + " WMRIGHT " + str(Wmright) + " "
    #     self.msg = self.msg + "CMDL " + str(CmdL) + " CMDR " + str(CmdR) + " "
    #     self.msg = self.msg + "THETA " + str(th) + " /"
    #
    #     if Kal:  # log kalman filter if it exist
    #         X = str(Kal.X.tolist())
    #         g = str(Kal.Gamma.tolist())
    #         U = str(Kal.u.tolist())
    #         Y = str(Kal.y.tolist())
    #         self.msg = self.msg + "KAL: STATE " + X + " COVARIANCE " + g + " INPUT " + U + " OUTPUT " + Y + " /"
    #     return

    def log_update_write(self):  # write and reset the message
        logging.info(self.msg)
        self.msg = ""
        return


# noinspection PyBroadException
def init_drivers(imu_mode=2,robot_id="0"):  # test connection to sensors and return sensor class
    print("initialising drivers")
    try:
        ard_ = ardu.ArduinoIO()
    except:
        ard_ = 0
        print("arduino interface not working")

    try:
        gps_ = gpsdrv.GpsIO()
    except:
        gps_ = 0
        print("gpx not connected")

    try:
        imu_ = imudv.Imu9IO()
        imu_.setup_accel_filter(imu_mode)
        print("loading compass_parameters_"+str(robot_id)+".json")
        imu_.load_calibration_parameters("compass/compass_parameters_"+str(robot_id)+".json")
    except:
        imu_ = 0
        print("imu not connected")
    print("drivers initialised")
    return ard_, gps_, imu_


if __name__ == "__main__":
    log = LogRecorder()

    ard, gps, imu = init_drivers(2,"5")
    print("drivers initialised")

    # can have a command line argument to set the motor command
    try:
        cmdl = int(sys.argv[1])
    except:
        cmdl = 0
    try:
        cmdr = int(sys.argv[2])
    except:
        cmdr = 0
    ard.send_arduino_cmd_motor(cmdl, cmdr)
    print("Logging on")

    while True:
        log.log_observe_update(ard, gps, imu)
        log.log_update_write()
        time.sleep(0.5)
