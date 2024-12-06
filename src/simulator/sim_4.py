# circle patrol around the big boat

import numpy as np

from boat import *
import time
from controller import *

speed = True

N = 10 # number of escort boats

# initial state
x0 = np.array([0., 0, 0])
Lx0 = []
for i in range(N):
    Lx0.append(np.array([-10, (i-N/2), 0]))


# references
circle_radius = 5
L_phi = []
for i in range(N):
    L_phi.append(2 * -i * np.pi / N)
w_patrol = 0.2


# create the boat
boat_master = Boat(x0,traj_memory_=-1)
L_escort = []
for i in range(N):
    L_escort.append(Boat(Lx0[i],traj_memory_=20, motor_limitation=True))

# simulation parameters
sim = SimuDisplay()
dt = 0.05  # time step
T = 50.0  # simulation duration
# N = int(T/dt)  # number of iterations

# a figure to display the speed of the robot
if speed:
    fig, ax2 = plt.subplots()
    ax2.set_xlim(0.1, L_escort[0].v1max*1.1)
    ax2.set_ylim(-L_escort[0].v2max*1.1, L_escort[0].v2max*1.1)

# simulation
t = 0
L_vd = []
for i in range(N):
    L_vd.append(np.array([0,0]))
p_dot_master = np.array([0,0])
while t < T:
    t1 = time.time()
    sim.clear()
    if speed:
        clear2(ax2)

        # plot the speed limits
        ax2.plot([L_escort[0].v1min, 0.5 * L_escort[0].v1max], [L_escort[0].v2min, L_escort[0].v2max], 'k--')
        ax2.plot([L_escort[0].v1min, 0.5 * L_escort[0].v1max], [-L_escort[0].v2min, -L_escort[0].v2max], 'k--')
        ax2.plot([2 * L_escort[0].v1min, 0.5 * L_escort[0].v1max + L_escort[0].v1min,
                  L_escort[0].v1max, 0.5 * L_escort[0].v1max + L_escort[0].v1min,
                  2 * L_escort[0].v1min],
                 [0, L_escort[0].v2max - L_escort[0].v2min, 0, -L_escort[0].v2max + L_escort[0].v2min, 0], 'k--')

    # master control input
    u = np.array([0,30])  # (left motor rotation speed, right motor rotation speed)

    # update the master state
    v_master = boat_master.motor_update(dt, u)
    p_dot_master_old = np.array(p_dot_master)
    p_dot_master = v_master * np.array([np.cos(boat_master.th()), np.sin(boat_master.th())])
    p_ddot_master = (p_dot_master - p_dot_master_old) / dt

    # display master
    sim.ax.add_artist(plt.Circle((boat_master.x[0], boat_master.x[1]), circle_radius, color='r', fill=False))
    draw_tank2(sim.ax,boat_master.x,"r",r=1)
    draw_traj(sim.ax,boat_master.traj,"r")

    for i in range(N):
        pd = boat_master.p() + circle_radius * np.array([np.cos(L_phi[i]+w_patrol*t), np.sin(L_phi[i]+w_patrol*t)])
        pd_dot = p_dot_master + circle_radius * w_patrol * np.array([-np.sin(L_phi[i]+w_patrol*t), np.cos(L_phi[i]+w_patrol*t)])
        pd_ddot = p_ddot_master +circle_radius * w_patrol**2 * np.array([-np.cos(L_phi[i]+w_patrol*t), -np.sin(L_phi[i]+w_patrol*t)])

        sim.ax.plot(pd[0], pd[1], 'k*', markersize=5)

        # control update
        L_vd[i] = follow_pose(pd,L_escort[i].x,dt,v1_old=L_vd[i][0],pd_dot=pd_dot,pd_ddot=pd_ddot)
        u = L_escort[i].convert_motor_control_signal(L_vd[i])

        # boat update
        v = L_escort[i].motor_update(dt, u)

        draw_tank2(sim.ax,L_escort[i].x,"k")
        draw_traj(sim.ax,L_escort[i].traj,"k")

        if speed:
            # plot the speed of the robot
            ax2.plot(v[0], v[1], 'ko', markersize=5)

            # desired speed
            ax2.plot(L_vd[i][0], L_vd[i][1], 'kx', markersize=10)

    # print time on the figure
    sim.ax.text(0.1, 0.9, 'Time: {:.2f}'.format(t), transform=sim.ax.transAxes)

    # update time
    t += dt
    plt.pause(0.01)

print("done")
plt.show()