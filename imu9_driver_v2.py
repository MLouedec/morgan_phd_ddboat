import struct
import time
import sys
import math
from sunau import AUDIO_FILE_ENCODING_LINEAR_32

import i2creal as i2c  # currently only real I2C on ddboats (no simulated I2C)
import json
import numpy as np

# LIS3DML 0x1e  (mag sensor)
# LSM6    0x6b  (accelero - gyro)

def sawtooth(x):
    # return (x + pi) % (2 * pi) - pi  # or equivalently   2*arctan(tan(x/2))
    return 2*np.arctan(np.tan(x/2))

class Imu9IO():
    def __init__(self):
        self.__bus_nb = 1  # 1 on DDBoat, 2 on DartV2
        self.__addr_mg = 0x1e  # mag sensor
        self.__addr_ag = 0x6b  # accelero - gyro

        self.__dev_i2c_mg=i2c.i2c(self.__addr_mg,self.__bus_nb)
        self.__dev_i2c_ag=i2c.i2c(self.__addr_ag,self.__bus_nb)

    # place your imu functions here
        self.magx_min = 0.0
        self.magx_max = 0.0
        self.magy_min = 0.0
        self.magy_max = 0.0
        self.magx_offs = 0.0
        self.magy_offs = 0.0
        self.magx_scale = 0.0
        self.magy_scale = 0.0

        self.__mag_raw = [0.0,0.0,0.0]
        self.__accel_raw = [0.0,0.0,0.0]
        self.__gyro_raw = [0.0,0.0,0.0]
        #
        # configure mag sensor
        # CTRL_REG1 (0x20) = 0b01110000
        # OM = 11 (ultra-high-performance mode for X and Y);
        # DO = 100 (10 Hz ODR)
        self.__dev_i2c_mg.write(0x20,[0x70])
        # CTRL_REG2 (0x21) = 0b00000000
        # FS = 00 (+/- 4 gauss full scale)
        self.__dev_i2c_mg.write(0x21,[0x00])
        # CTRL_REG3 (0x22) = 0b00000000
        # MD = 00 (continuous-conversion mode)
        self.__dev_i2c_mg.write(0x22,[0x00])
        # CTRL_REG4 (0x23) = 0b00001100
        # OMZ = 11 (ultra-high-performance mode for Z)
        self.__dev_i2c_mg.write(0x23,[0x0C])
        #
        # configure accelero + gyro
        # LSM6DS33 gyro
        # CTRL2_G (0x11) = 0b10001100
        # ODR = 1000 (1.66 kHz (high performance))
        # FS_G = 11 (2000 dps)
        self.__dev_i2c_ag.write(0x11,[0x8C])
        #
        # CTRL7_G (0x16) = 0b00000000
        # defaults
        self.__dev_i2c_ag.write(0x16,[0x00])
        #
        # LSM6DS33 accelerometer
        # CTRL1_XL (0x10) = 0b10001100
        # ODR = 1000 (1.66 kHz (high performance))
        # FS_XL = 11 (8 g full scale)
        # BW_XL = 00 (400 Hz filter bandwidth)
        #self.__dev_i2c_ag.write(0x10,[0x8C])
        # more filtering BW_XL = 11 (50 Hz filter bandwidth)
        self.__dev_i2c_ag.write(0x13,[0x00])
        self.__dev_i2c_ag.write(0x10,[0x8C])
        #
        # common
        # CTRL3_C (0x12) 0b00000100
        # IF_INC = 1 (automatically increment address register)
        self.__dev_i2c_ag.write(0x12,[0x04])

        # calibration parameters
        self.b = np.zeros(3)
        self.A = np.eye(3)
        self.R_imu = np.eye(3)

    def setup_accel_filter (self,mode):
        if mode == 0:
            self.__dev_i2c_ag.write(0x17,[0x00])
            self.__dev_i2c_ag.write(0x13,[0x00])
            self.__dev_i2c_ag.write(0x10,[0x8C])
        elif mode == 1:
            self.__dev_i2c_ag.write(0x17,[0x00])
            self.__dev_i2c_ag.write(0x13,[0x80])
            self.__dev_i2c_ag.write(0x10,[0x8F])
        elif mode == 2:
            self.__dev_i2c_ag.write(0x17,[0x80])
            self.__dev_i2c_ag.write(0x13,[0x80])
            self.__dev_i2c_ag.write(0x10,[0x8F])
                    
    def read_mag_raw(self):
        v = self.__dev_i2c_mg.read(0x28,6)
        ix = self.cmpl2(v[0],v[1])
        iy = self.cmpl2(v[2],v[3])
        iz = self.cmpl2(v[4],v[5])
        self.__mag_raw = [ix,iy,iz]
        return self.__mag_raw
 
    def read_gyro_raw(self):
        # OUTX_L_G (0x22)
        v = self.__dev_i2c_ag.read(0x22,6)
        ix = self.cmpl2(v[0],v[1])
        iy = self.cmpl2(v[2],v[3])
        iz = self.cmpl2(v[4],v[5])
        self.__gyro_raw = [ix,iy,iz]
        return self.__gyro_raw
 
    def read_accel_raw(self):
        # OUTX_L_XL (0x28)
        v = self.__dev_i2c_ag.read(0x28,6)
        ix = self.cmpl2(v[0],v[1])
        iy = self.cmpl2(v[2],v[3])
        iz = self.cmpl2(v[4],v[5])
        self.__accel_raw = [ix,iy,iz]
        return self.__accel_raw

    def cmpl2(self,lsByte,msByte):
        i = lsByte + (msByte << 8)
        if i >= (1<<15):
            i = i - (1<<16)
        return i

    def heading_raw(self,magx,magy):
        heading = math.atan2(magy,magx)
        return heading
        
    def heading_raw_deg(self,magx,magy):
        heading = self.heading_raw(magx,magy)*180.0/math.pi
        if heading < 0.0:
            heading += 360.0
        return heading

    # def fast_heading_calibration (self, magx_min, magx_max, magy_min, magy_max):
    #     self.magx_min = magx_min
    #     self.magx_max = magx_max
    #     self.magy_min = magy_min
    #     self.magy_max = magy_max
    #     self.magx_offs = (magx_min+magx_max)/2.0
    #     self.magy_offs = (magy_min+magy_max)/2.0
    #     self.magx_scale = 2./(magx_max-magx_min)
    #     self.magy_scale = 2./(magy_max-magy_min)
        
    # def heading(self,magx,magy):
    #     magx_cal = (magx - self.magx_offs) * self.magx_scale
    #     magy_cal = (magy - self.magy_offs) * self.magy_scale
    #     #print (self.magx_min,magx,self.magx_max,self.magy_min,magy,self.magy_max,magx_cal,magy_cal)
    #     magx_cal = 1.0 if magx_cal>1.0 else magx_cal
    #     magx_cal = -1.0 if magx_cal<-1.0 else magx_cal
    #     magy_cal = 1.0 if magy_cal>1.0 else magy_cal
    #     magy_cal = -1.0 if magy_cal<-1.0 else magy_cal
    #     heading = math.atan2(magy_cal,magx_cal)
    #     return heading

    def heading(self, mag_cal,pitch,roll):
        # take into acount the pitch and the roll
        # pitch and roll are in rad
        # magx_cal and magy_cal are the corrected magnetometer data
        # the function returns the heading in rad

        R_pitch = np.array([[np.cos(pitch),0,np.sin(pitch)],
                            [0,1,0],
                            [-np.sin(pitch),0,np.cos(pitch)]])
        R_roll = np.array([[1,0,0],
                            [0,np.cos(roll),-np.sin(roll)],
                            [0,np.sin(roll),np.cos(roll)]])
        mag2 = R_pitch@R_roll@mag_cal

        heading = sawtooth(math.atan2(mag2[1],-mag2[0]))
        return heading  # rad

    def heading_simple(self,mag_cal):
        # don't take into account the pitch and the roll
        heading = math.atan2(mag_cal[1],-mag_cal[0])
        return heading

    def heading_deg(self,mag_cal,pitch,roll):
        heading = self.heading(mag_cal,pitch,roll)*180.0/math.pi
        if heading < 0.0:
            heading += 360.0
        return heading

    def load_calibration_parameters(self,filename):
        # load the mag calibration parameters b and A
        with open(filename) as f:
            data = json.load(f)
            self.b = data["b"]
            self.A = np.array(data["A"]).reshape(3,3)
            # self.heading_N = data["heading_N"] # heading of the north in deg
            # self.heading_N_rad = self.heading_N*math.pi/180.0
            print("loaded calibration parameters")
            print("b:",self.b)
            print("A:",self.A)

            mag_x = np.array(data["mag_x"])
            mag_y = np.array(data["mag_y"])
            mag_z = np.array(data["mag_z"])

            self.R_imu = np.eye(3)
            mag_x_cal = self.cal_mag(mag_x)
            mag_y_cal = self.cal_mag(mag_y)
            mag_z_cal = self.cal_mag(mag_z)

            x = mag_x_cal/np.linalg.norm(mag_x_cal)
            y = mag_y_cal/np.linalg.norm(mag_y_cal)
            z = mag_z_cal/np.linalg.norm(mag_z_cal)

            print("x:",x)
            print("y:",y)
            print("z:",z)
            M = np.array([x,y,z]).T
            M = 0.5*(M+M.T)
            print("M:",M)
            self.R_imu = np.linalg.inv(M)
            print("R_imu:",self.R_imu)

    def cal_mag(self,mag):
        # mag is the raw magnetometer data
        # the function returns the calibrated magnetometer data
        c_mag = np.array(mag)
        c_mag = c_mag - self.b
        c_mag = self.R_imu@np.dot(self.A,c_mag)
        return c_mag

    def get_pitch_roll(self):
        # use the accelerometer to evaluate the pitch and roll
        # the vibration must be fitered before calling this function
        accel  = self.R_imu@self.read_accel_raw()
        pitch = math.atan2(accel[0],math.sqrt(accel[1]*accel[1]+accel[2]*accel[2]))
        roll = math.atan2(-accel[1],math.sqrt(accel[0]*accel[0]+accel[2]*accel[2])) # axis inverted
        return pitch,roll


    
                    
if __name__ == "__main__":
    imu = Imu9IO()
    imu.setup_accel_filter(0)
    imu.load_calibration_parameters("compass/compass_parameters.json")
    for i in range(2000):
        raw_mag = imu.read_mag_raw()
        print ("\nraw_data:",raw_mag,imu.read_accel_raw(),imu.read_gyro_raw())
        c_mag = imu.cal_mag(raw_mag)
        print("calibrated_mag:",c_mag)

        pitch,roll = imu.get_pitch_roll()
        print("pitch:",pitch*180.0/math.pi)
        print("roll:",roll*180.0/math.pi)

        heading = imu.heading(c_mag,pitch,roll)
        print("heading:",heading)
        print("heading_deg:",imu.heading_deg(c_mag,pitch,roll))
        time.sleep(0.5)

