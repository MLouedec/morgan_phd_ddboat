# Simple mission, round trim between the original point and a target

from mission_param import *

mp = MissionBlock(rh=True)

# init variables
k, pd0, pos_old, t_pos_old, cmdL, cmdR = 0, mp.CB.pd0, mp.kal.p(), time.time(), 0, 0
step = 1

# get desired position (pd0 : initial position, pd1: desired position)
lat1 = int(sys.argv[1])
lon1 = int(sys.argv[2])
pd1 = mp.filt.latlon_to_coord(lat1, lon1)
pd = pd1

while time.time() < mp.time_mission_max:

    mp.measure(cmdL, cmdR)  # measurement

    # reference update
    if step == 1 and np.linalg.norm(mp.kal.p() - pd) < 2:  # return home when reaching target
        pd = pd0
        step += 1
    elif step == 2 and np.linalg.norm(mp.kal.p() - pd) < 2:
        print("end of the mission")
        break

    # control update
    cmdL, cmdR, u = mp.CB.follow_reference2(pd)
    mp.ard.send_arduino_cmd_motor(cmdL, cmdR)
    mp.log_rec.log_control_update(u[0, 0], u[1, 0], mp.wmLeft, mp.wmRight, cmdL, cmdR, pd, mp.y_th, mp.kal)
    mp.kal.Kalman_update(0, mp.y_th)
    mp.log_rec.log_update_write()  # write in the log file

    # loop update
    if not mp.sync:
        print("arduino communication lost, break !")
        break
    if time.time() > mp.time_mission_max:
        print("maximum allowed time passed, breaking !")
        break

    t_execution = time.time() - mp.t
    delta_t = mp.dt - t_execution
    if delta_t > 0:
        time.sleep(delta_t)
    else:
        print("LAG loop frequency reduced, t_execution ", t_execution)

mp.ard.send_arduino_cmd_motor(0, 0)
