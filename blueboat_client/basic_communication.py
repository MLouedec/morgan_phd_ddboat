# let us communicate with a blue boat using the Pymavlink library

#  GCS Client Link ardupilot-manager
# UDP Client 192.168.2.19:14550

from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import time
import math
import asyncio

class BluebotCom:
    def __init__(self):
        # connect to blueboat autopilot (Ip adresse of the raspberry)
        self.master = mavutil.mavlink_connection('udpin:192.168.2.19:14550')

        print("Waiting for HEARTBEAT")
        self.master.wait_heartbeat()
        self.boot_time = time.time()
        print("HEARTBEAT received - connection successful")

        self.set_control_mode()

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.GPS_ON = False
        self.altitude = 0
        self.lng = 0
        self.lat = 0

        self.voltage = 0
        # self.timestamp = ""

        self.tc1 = 0
        self.ts1 = 0

        self.autopilot_mode =""

    def arm(self):
        # self.master.arducopter_arm()
        # self.master.master.motors_armed_wait()
        # Arm the drone
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        print("Arming command sent")

        time.sleep(1)

    def disarm(self):
        self.master.arducopter_disarm()
        self.master.master.motors_disarmed_wait()

    def set_control_mode(self,mode="MANUAL"):
        if mode == "MANUAL":
            # set to Manual Mode (thurst)
            mode_id = self.master.mode_mapping()["MANUAL"]
        else:
            print("Unknown mode")
            print("Available modes are:")
            print(self.master.mode_mapping())
            return

        self.master.mav.set_mode_send(self.master.target_system,
                                 mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                 mode_id)
        # master.mav.set_mode_send(master.target_system, mavlink1.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15)
        # Loiter 12
        # Guided 15


    def send_arm_command(self):
        self.master.mav.command_long_send(self.master.target_system,
                                          self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                          1, 1, 0, 0, 0, 0, 0, 0)

    def send_disarm_command(self):
        self.master.mav.command_long_send(self.master.target_system,
                                          self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                          0, 0, 0, 0, 0, 0, 0, 0)


    def send_desired_heading(self,dth):
        # Send a message to the server
        message = "d_th:" + str(dth)
        self.master.mav.statustext_send(1, message)

    def get_AHRS(self):
        msg = self.master.recv_match(type='AHRS2', blocking=False)
        if not msg:
            return
        if msg.get_type() == "AHRS2":
            self.roll = msg.roll
            self.pitch = msg.pitch
            self.yaw = msg.yaw

            self.lat = msg.lat
            self.lng = msg.lng
            self.altitude = msg.altitude

            # check if GPS is on
            if not self.GPS_ON:
                if self.lat * self.lng * self.altitude != 0:
                    self.GPS_ON = True

        else:
            print("bad message type")

    def get_status(self):
        msg = self.master.recv_match(type='SYS_STATUS', blocking=False)
        if not msg:
            return
        if msg.get_type() == "SYS_STATUS":
            self.voltage = msg.voltage_battery
            # self.timestamp = msg.timestamp
        else:
            print("bad message type")

    def get_time(self):
        msg = self.master.recv_match(type='TIMESYNC', blocking = False)
        if not msg:
            return

        if msg.get_type() == "TIMESYNC":
            self.tc1 = msg.tc1
            self.ts1 = msg.ts1
            # self.timestamp = msg.timestamp
        else:
            print("bad message type")

    def send_control(self,cmd):
        # select the right rc mode
        # send a RC_CHANNELS_OVERRIDE message
        # pitch, roll, throttle, yaw, forward and lateral
        self.master.mav.rc_channels_override_send(
            self.master.target_system, self.master.target_component, *cmd)

    def set_target_attitude(self, roll, pitch, yaw):
        """ Sets the target attitude while in depth-hold mode.
        'roll', 'pitch', and 'yaw' are angles in radians
        """
        self.master.mav.set_attitude_target_send(
            int(1e3 * (time.time() - self.boot_time)),  # ms since boot
            self.master.target_system, self.master.target_component,
            # allow throttle to be controlled by depth_hold mode
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
            # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
            QuaternionBase([angle for angle in (roll, pitch, yaw)]),
            0, 0, 0, 0  # roll rate, pitch rate, yaw rate, thrust
        )

M = BluebotCom()
try:
    M.send_arm_command()
    while True:
        # M.send_desired_heading(0)
        M.get_AHRS()
        M.get_status()
        M.get_time()
        # print("\ntimestamp:",M.timestamp)
        print("yaw:",M.yaw)
        print("battery voltage (mV)",M.voltage)
        print("ts1:",M.ts1)

        # thrust = 1.
        time.sleep(1)
except KeyboardInterrupt:
    print("BluebotCom Stopped ")

finally:
    M.send_disarm_command()
