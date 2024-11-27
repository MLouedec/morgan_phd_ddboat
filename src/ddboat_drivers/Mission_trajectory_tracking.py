# main script of the trajectory tracking mission, the robot must follow a desired moving target

from DDBOAT_mission import *

mp = MissionBlock(rh=True)
# init variables
k, pd, pos_old, t_pos_old, cmdL, cmdR = 0, mp.CB.pd0, mp.kal.p(), time.time(), 0, 0
wait_for_signal = True  # after time mission begin, the robot start following the trajectory

while time.time() < mp.time_mission_max:

    mp.measure(cmdL,cmdR)

    # control update
    if wait_for_signal:  # stand by initial waypoint
        cmdL, cmdR, u = mp.CB.station_keeping1(mp.t)
        if time.time() > mp.time_mission_begin:  # after time mission begin, the robot start following the trajectory
            wait_for_signal = False
            print("Start following the trajectory")
    else:  # follow reference

        # update reference
        try:
            traj_k = mp.traj[k]
            pd = np.reshape(np.array([traj_k["pd"]]), (2, 1))
            pd_dot = np.reshape(np.array([traj_k["pd_dot"]]), (2, 1))
            pd_ddot = np.reshape(np.array([traj_k["pd_ddot"]]), (2, 1))
            k += 1
        except:  # end of the trajectory
            if mp.return_home and np.linalg.norm(mp.home_pos-mp.kal.p())>20:
                print("end of the trajectory, return home")
                pd = mp.home_pos
                pd_dot, pd_ddot = np.zeros((2,1)), np.zeros((2,1))
            else:
                print("end of the mission, break !")
                break

        cmdL, cmdR, u = mp.CB.follow_reference2(pd, pd_dot, pd_ddot)
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
