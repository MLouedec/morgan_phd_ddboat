# square mission based on heading from Guerledan teaching

from mission_param import *

mp = MissionBlock(rh=True) # the trajectory is useless
# init variables
k, pd, pos_old, t_pos_old, cmdL, cmdR = 0, mp.CB.pd0, mp.kal.p(), time.time(), 0, 0
print("Starting the square mission")
wait_for_signal = False  # after time mission begin, the robot start following the trajectory

heading_list = [0,np.pi/2,np.pi,-np.pi]
time_init = time.time()
while time.time() < mp.time_mission_max:
    mp.measure(cmdL, cmdR)
    # update reference
    t = time.time()-time_init
    i = int(t//30) # turn every 30 second
    try:
        th_d = heading_list[i]
    except:
        break # mission finished
    v_d = 0.5 # m/s
    print("th_d",th_d)

    # update heading controller
    cmdL, cmdR, w = heading_regul(v_d, th_d, mp.y_th,mp.wmLeft, mp.wmRight, mp.CB.cmdL_old, mp.CB.cmdR_old,mp.dt)
    mp.ard.send_arduino_cmd_motor(cmdL, cmdR)
    mp.log_rec.log_control_update(th_d, w, mp.wmLeft, mp.wmRight, cmdL, cmdR, np.zeros((2,1)), mp.y_th, mp.kal) # note w and th_d replace u[0,0] and u[1,0]
    mp.kal.Kalman_update(np.zeros((2,1)), mp.y_th)
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
