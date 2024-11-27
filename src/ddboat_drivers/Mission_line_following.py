# main script of the line following mission with GNSS reference waypoints

from DDBOAT_mission import *

mp = MissionBlock(rh=True) # trajectory is useless
# init variables
k, pd, pos_old, t_pos_old, cmdL, cmdR = 0, mp.CB.pd0, mp.kal.p(), time.time(), 0, 0

waypoint_list = np.array([[48.199024,48.199038,48.199817],[-3.014790,-3.015807,-3.015603]]) # home, west, north-west
print("waypoint list:",waypoint_list)

# convert waypoint to local coordinate
for i in range(len(waypoint_list[0])):
    pos = mp.filt.latlon_to_coord(waypoint_list[0,i], waypoint_list[1,i])
    waypoint_list[:,[i]] = pos
print("waypoint list local:",waypoint_list)

while time.time() < mp.time_mission_max:

    mp.measure(cmdL,cmdR)
    print("current position:",mp.kal.p().T)

    # test change of line
    a = waypoint_list[:, [k]]
    b = waypoint_list[:, [k + 1]]
    if np.dot((b-a).flatten(),(mp.kal.p()-b).flatten())>0: # if the boat has passed the end of the line
        print("next line")
        k +=1
        if k>(len(waypoint_list[0])-2):
            print("mission done")
            break
    print(" ")
    # update reference
    th_d = follow_line(a, b, mp.kal.p())
    v_d = 5 # m/s
    
    print("th",mp.y_th)
    print("th_d:",th_d)
    print("b-a",(b-a).T)
    print("m-a",(mp.kal.p()-a).T)

    cmdL, cmdR, w = heading_regul(v_d, th_d, mp.y_th,mp.wmLeft, mp.wmRight, mp.CB.cmdL_old, mp.CB.cmdR_old,mp.dt)
    mp.ard.send_arduino_cmd_motor(cmdL, cmdR)
    mp.log_rec.log_control_update(th_d, w, mp.wmLeft, mp.wmRight, cmdL, cmdR, b, mp.y_th, mp.kal) # note w and th_d replace u[0,0] and u[1,0]
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
