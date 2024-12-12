import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time
from utilities import *

#Global variables
realHeading = 0

class DDBoat:
    def __init__(self, id, position=None, heading=None):
        self.id = id
        self.position = np.random.rand(2) * 10  
        self.heading = np.random.rand() * 2 * np.pi  

    def draw(self, ax):
        triangle = self.get_triangle()
        ax.fill(triangle[:, 0], triangle[:, 1], color='b')

    def get_triangle(self):
        L = 0.5  
        triangle_vertices = np.array([
            [2*L, 0],    
            [-L / 2, (np.sqrt(3) / 2) * L],
            [-L / 2, -(np.sqrt(3) / 2) * L]
        ])

        triangle_edges = np.array([
            triangle_vertices[0], triangle_vertices[1],  
            triangle_vertices[1], triangle_vertices[2],
            triangle_vertices[2], triangle_vertices[0] 
        ])
        print(self.heading)
        rotation_matrix = np.array([
            [np.cos(self.heading), -np.sin(self.heading)],
            [np.sin(self.heading), np.cos(self.heading)]
        ])

        rotated_triangle = (triangle_edges @ rotation_matrix.T) + self.position
        return rotated_triangle

class DDBoatR(DDBoat):
    def updateHeading(self, head):
        client = connectSocket()
        message = "d_th:" + str(int(head*180/np.pi))
        client.send(message.encode())
        response = client.recv(1024).decode()
        try:
            global realHeading
            realHeading = float(response)
        except:
            print("empty response")
        

class DDBoatV(DDBoat):
    pass


class Network:
    def __init__(self, boats, consensus_rate):
        self.boats = boats
        self.consensus_rate = consensus_rate

    def update_consensus(self):
        headings = np.array([boat.heading for boat in self.boats])
        avg_heading = np.mean(headings)  
        for boat in self.boats:
            if(isinstance(boat, DDBoatR)):
                thread = threading.Thread(target=boat.updateHeading(avg_heading))
                thread.start()
                global realHeading
                boat.heading = realHeading
            else:
                boat.heading += self.consensus_rate * sawtooth(avg_heading - boat.heading)  

class Visualization:
    def __init__(self, boats):
        self.boats = boats
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-1, 15)
        self.ax.set_ylim(-1, 15)
        self.drone_patches = [None] * len(boats)

    def init(self):
        for i, boat in enumerate(self.boats):
            if(isinstance(boat, DDBoatR)):
                self.drone_patches[i], = self.ax.plot([], [], 'r')
            else:
                self.drone_patches[i], = self.ax.plot([], [], 'b')
        return self.drone_patches

    def update(self, frame):
        network.update_consensus()
        # for boat in self.boats:
        #     boat.move(velocity, time_step) 
        return self.draw_boats()

    def draw_boats(self):
        for i, boat in enumerate(self.boats):
            triangle = boat.get_triangle()
            self.drone_patches[i].set_data(triangle[:, 0], triangle[:, 1])
        return self.drone_patches

num_boats = 5
consensus_rate = 0.05
# time_step = 0.2
# velocity = 0


boats = [DDBoat(i) for i in range(num_boats)]
boats.append(DDBoatR(5))


network = Network(boats, consensus_rate)

visualization = Visualization(boats)

ani = FuncAnimation(visualization.fig, visualization.update, frames=200, init_func=visualization.init, blit=True, interval=50)

plt.show()
