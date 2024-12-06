# sim_2 simulating different point tracking strategies
# red robot simple point tracking with constant speed
# blue robot same but has speed proportional to the distance to the target
# green robot max speed with station keeping v0
# --- black robot station keeping v1
# you can tune the current to see the effect on the robots

import numpy as np

from boat import *
import time
from controller import *

# current
drift = 0*np.array([0.1, -0.1])

# initial state
x_red = np.array([-5., -5, 0])
x_blue = np.array([-5., -5, 0])
x_green = np.array([-5., -5, 0])
x_black = np.array([-5., -5, 0])

# create the boat
boat_red = Boat(x_red,traj_memory_=-1)
boat_blue = Boat(x_blue,traj_memory_=-1)
boat_green = Boat(x_green,traj_memory_=-1)
# boat_black = Boat(x_black,traj_memory_=-1)

# references
speed_red = 0.49*boat_red.v1max # desired speed for the green boat
pd = np.array([0.5, -0.7]) # desired position

speed_green = speed_red
d_station_keeping = 2*boat_green.r_min # max distance to the station

# SK = StationKeeping()
speed_black  = speed_red

# simulation parameters
sim = SimuDisplay()
dt = 0.05  # time step
T = 100.0  # simulation duration
N = int(T/dt)  # number of iterations

# a figure to display the speed of the robot
fig, ax2 = plt.subplots()
ax2.set_xlim(-0.1, boat_red.v1max*1.1)
ax2.set_ylim(-boat_red.v2max*1.1, boat_red.v2max*1.1)

# simulation
t = 0
vd_blue = np.array([0,0])
while t < T:
    t1 = time.time()
    sim.clear()
    clear2(ax2)

    # control input
    # thd_red = follow_point(pd, boat_red.p())
    # vd_red = heading_regul(speed_red, thd_red, boat_red.th())
    # u_red = boat_red.convert_motor_control_signal(vd_red)

    vd_blue = follow_pose(pd,boat_blue.x,dt,v1_old=vd_blue[0])
    u_blue = boat_blue.convert_motor_control_signal(vd_blue)

    # vd_green = control_station_keeping(pd, d_station_keeping, boat_green.p(), boat_green.th(), speed_green)
    # u_green = boat_green.convert_motor_control_signal(vd_green)

    # vd_black = SK.station_keeping1(pd, boat_black.p(), boat_black.th(), speed_red, t,r=d_station_keeping)
    # u_black = boat_black.convert_motor_control_signal(vd_black)

    # update the state
    # v_red = boat_red.motor_update(dt, u_red,drift)
    v_blue = boat_blue.motor_update(dt, u_blue,drift)
    # v_green = boat_green.motor_update(dt, u_green,drift)
    # boat_black.motor_update(dt, u_black,drift)

    # display
    sim.ax.plot(pd[0], pd[1], 'ko', markersize=10)

    # draw circle of station keeping (center pd, radius d_station_keeping)
    sim.ax.add_artist(plt.Circle((pd[0], pd[1]), d_station_keeping, color='k', fill=False))
    sim.ax.add_artist(plt.Circle((pd[0], pd[1]), 0.5*d_station_keeping, color='k', linestyle="--", fill=False))

    draw_tank2(sim.ax,boat_red.x,"r")
    draw_traj(sim.ax,boat_red.traj,"r")

    draw_tank2(sim.ax,boat_blue.x,"b")
    draw_traj(sim.ax,boat_blue.traj,"b")

    draw_tank2(sim.ax,boat_green.x,"g")
    draw_traj(sim.ax,boat_green.traj,"g")

    # draw_tank2(sim.ax, boat_black.x, "k")
    # draw_traj(sim.ax, boat_black.traj, "k")

    # plot the speed limits
    ax2.plot([boat_red.v1min, 0.5*boat_red.v1max], [boat_red.v2min, boat_red.v2max], 'k--')
    ax2.plot([boat_red.v1min, 0.5*boat_red.v1max], [-boat_red.v2min, -boat_red.v2max], 'k--')
    ax2.plot([2*boat_red.v1min, 0.5 * boat_red.v1max+boat_red.v1min,
              boat_red.v1max,0.5 * boat_red.v1max+boat_red.v1min,
              2*boat_red.v1min],
             [0, boat_red.v2max-boat_red.v2min,0,-boat_red.v2max+boat_red.v2min,0], 'k--')
    # plot the speed of the robot
    # ax2.plot(v_red[0], v_red[1], 'ro', markersize=5)
    ax2.plot(v_blue[0], v_blue[1], 'bo', markersize=5)
    # ax2.plot(v_green[0], v_green[1], 'go', markersize=5)

    # desired speed
    # ax2.plot(vd_red[0], vd_red[1], 'rx', markersize=10)
    ax2.plot(vd_blue[0], vd_blue[1], 'bx', markersize=10)
    # ax2.plot(vd_green[0], vd_green[1], 'gx', markersize=10)

    # print time on the figure
    sim.ax.text(0.1, 0.9, 'Time: {:.2f}'.format(t), transform=sim.ax.transAxes)

    # update time
    t += dt
    plt.pause(0.01)

print("done")
plt.show()