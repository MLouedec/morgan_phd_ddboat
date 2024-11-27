# square mission based on heading from Guerledan teaching

from DDBOAT_mission import *
import sys

if __name__ == "__main__":
    # step 1: load the mission parameters
    try:
        mission_filename = sys.argv[1]
        MB = MissionBlock(mission_filename)

        # Log
        LOG = LogRecorder(MB.local_time_mission_begin,MB.robot_id)
    except:
        print("mission file not found, stop the scipt")
        sys.exit()

    try:
        # step 2: wait for the beginning of the mission
        MB.wait_for_mission_begin()

        # step 3: mission execution
        init_time = time.time()
        while True:
            t1 = time.time()
            if t1-init_time > MB.time_mission_max:
                break
            MB.measure()
            MB.control()
            MB.log_rec.log_update_write()

            t2 = time.time()
            if t2 - t1 < MB.dt:
                time.sleep(MB.dt - (t2 - t1))

    except KeyboardInterrupt:
        print("Server shutting down.")
        shutdown_flag = True
    finally:
        MB.DS.server_socket.close() # Close the server socket to unblock accept()
        MB.DS.accept_thread.join() # Wait for the accept thread to finish
        print("Server shut down.")


mp = MissionBlock(rh=True) # the trajectory is useless
# init variables
k, pd, pos_old, t_pos_old, cmdL, cmdR = 0, mp.CB.pd0, mp.kal.p(), time.time(), 0, 0
print("Starting the square mission")
wait_for_signal = False  # after time mission begin, the robot start following the trajectory

heading_list = [np.pi/2,np.pi,-np.pi/2,0] # 0 is east
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
    v_d = 5 # m/s
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
