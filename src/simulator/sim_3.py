# test lissajou point tracking
# red robot simple point tracking with constant speed
# blue robot reduce the speed when approaching the target
# green robot same with an integral term in the speed control
# you can tune the current to see the effect on the robots

import numpy as np

from boat import *
import time
from controller import *

speed = False

# current
drift = 0*np.array([0.1, -0.1])

# initial state
x0 = np.array([0., 0, 0])

# create the boat
boat_red = Boat(x0,traj_memory_=-1)
boat_blue = Boat(x0, traj_memory_=-1)
boat_green = Boat(x0, traj_memory_=-1)
boat_black = Boat(x0, traj_memory_=-1)

# references
speed_red = 0.49*boat_red.v1max
c = np.zeros(2)
R = 7
T_lissajou = 50
i = 0



# simulation parameters
sim = SimuDisplay()
dt = 0.05  # time step
T = 100.0  # simulation duration
N = int(T/dt)  # number of iterations

# a figure to display the speed of the robot
if speed:
    fig, ax2 = plt.subplots()
    ax2.set_xlim(-0.1, boat_red.v1max*1.1)
    ax2.set_ylim(-boat_red.v2max*1.1, boat_red.v2max*1.1)

# simulation
t = 0
# integral_green = 0
# integral_black = 0
vd_green = np.array([0,0])
vd_blue = np.array([0,0])
vd_black = np.array([0,0])
while t < T:
    t1 = time.time()
    sim.clear()

    if speed: clear2(ax2)

    # control input
    pd, pd_dot, pd_ddot = lissajou_trajectory(c, R, T_lissajou, t, i, N)

    # thd_red = follow_point(pd, boat_red.p())
    # vd_red = heading_regul(speed_red, thd_red, boat_red.th())
    # u_red = boat_red.convert_motor_control_signal(vd_red)

    vd_blue = follow_pose(pd,boat_blue.x,dt,v1_old=vd_blue[0])
    u_blue = boat_blue.convert_motor_control_signal(vd_blue)

    vd_green = follow_pose(pd,boat_green.x,dt,v1_old=vd_green[0],pd_dot=pd_dot)
    u_green = boat_green.convert_motor_control_signal(vd_green)

    vd_black = follow_pose(pd,boat_black.x,dt,v1_old=vd_black[0],pd_dot=pd_dot,pd_ddot=pd_ddot)
    u_black = boat_black.convert_motor_control_signal(vd_black)


    # update the state
    # v_red = boat_red.motor_update(dt, u_red,drift)
    v_blue = boat_blue.motor_update(dt, u_blue,drift)
    v_green = boat_green.motor_update(dt, u_green,drift)
    v_black = boat_black.motor_update(dt, u_black,drift)

    # display
    sim.ax.plot(pd[0], pd[1], 'ko', markersize=10)

    # draw p_dot
    sim.ax.arrow(pd[0], pd[1], pd_dot[0], pd_dot[1], head_width=0.5, head_length=0.5, fc='k', ec='k')

    # draw p_ddot
    sim.ax.arrow(pd[0], pd[1], pd_ddot[0], pd_ddot[1], head_width=0.5, head_length=0.5, fc='r', ec='r')

    # draw_tank2(sim.ax,boat_red.x,"r")
    # draw_traj(sim.ax,boat_red.traj,"r")

    draw_tank2(sim.ax,boat_blue.x,"b")
    draw_traj(sim.ax,boat_blue.traj,"b")

    draw_tank2(sim.ax,boat_green.x,"g")
    draw_traj(sim.ax,boat_green.traj,"g")

    draw_tank2(sim.ax, boat_black.x, "k")
    draw_traj(sim.ax, boat_black.traj, "k")

    if speed:
        # plot the speed limits
        ax2.plot([boat_red.v1min, 0.5 * boat_red.v1max], [boat_red.v2min, boat_red.v2max], 'k--')
        ax2.plot([boat_red.v1min, 0.5 * boat_red.v1max], [-boat_red.v2min, -boat_red.v2max], 'k--')
        ax2.plot([2 * boat_red.v1min, 0.5 * boat_red.v1max + boat_red.v1min,
                  boat_red.v1max, 0.5 * boat_red.v1max + boat_red.v1min,
                  2 * boat_red.v1min],
                 [0, boat_red.v2max - boat_red.v2min, 0, -boat_red.v2max + boat_red.v2min, 0], 'k--')
        # plot the speed of the robot
        # ax2.plot(v_red[0], v_red[1], 'ro', markersize=5)
        ax2.plot(v_blue[0], v_blue[1], 'bo', markersize=5)
        ax2.plot(v_green[0], v_green[1], 'go', markersize=5)
        ax2.plot(v_black[0], v_black[1], 'ko', markersize=5)

        # desired speed
        # ax2.plot(vd_red[0], vd_red[1], 'rx', markersize=10)
        ax2.plot(vd_blue[0], vd_blue[1], 'bx', markersize=10)
        ax2.plot(vd_green[0], vd_green[1], 'gx', markersize=10)
        ax2.plot(vd_black[0], vd_black[1], 'kx', markersize=10)

    # print time on the figure
    sim.ax.text(0.1, 0.9, 'Time: {:.2f}'.format(t), transform=sim.ax.transAxes)

    # update time
    t += dt
    plt.pause(0.01)

print("done")
plt.show()