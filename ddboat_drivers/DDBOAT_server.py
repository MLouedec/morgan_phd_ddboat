import socket
from DDBOAT_log import *
import threading
import time
import numpy as np

from DDBOAT_controller import sawtooth

class Memory:
    # shared variable for the multiple threads
    def __init__(self):
        self.lock = threading.Lock()

        self.cmdl_g, self.cmdr_g, self.roll_g, self.pitch_g, self.heading_g = 0., 0., 0., 0., 0.
        self.pos_g = np.array([[0., 0.]]).T

        # control reference
        self.heading_d = 0
        self.pos_d =  np.array([[0., 0.]]).T
        self.speed_d = 0
        self.rotation_speed_d = 0

        self.control_mode = "Standby"

    def current_goal(self):
        with self.lock:
            print("\nCurrent goal:")
            if self.control_mode == "Standby":
                print("Standby mode")
            elif self.control_mode == "Heading":
                print("Heading mode")
                print("Desired heading: ", self.heading_d)
                print("Desired speed: ",  self.speed_d)
            elif self.control_mode == "Waypoint":
                print("Waypoint mode")
                print("Desired position: ",  self.pos_d)
                print("Desired speed: ",  self.speed_d)
            elif self.control_mode == "Speed":
                print("Speed mode")
                print("Desired speed: ",  self.speed_d)
                print("Desired rotation speed: ",  self.rotation_speed_d)

memory = Memory() # general memory, global class variable

class DdboatServer:
    def __init__(self):
        # Server configuration

        # get the current IP address of the computer
        self.SERVER_HOST = socket.gethostbyname(socket.gethostname())
        # print("The IP address of this computer is: {}".format(self.SERVER_HOST))
        self.SERVER_HOST = "172.19.146.205"  # IP address of this computer
        self.SERVER_PORT = 5000  # Use any available port number

        # Create the server socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.SERVER_HOST, self.SERVER_PORT))
        self.server_socket.listen(1)  # Listen for 1 connection
        self.server_socket.settimeout(1.0) # Set a timeout for accept() to check shutdown flag

        print("Server listening on {}:{}".format(self.SERVER_HOST, self.SERVER_PORT))

        self.shutdown_flag = False  # Flag to signal the server to shut down

        # Shared memory
        # self.memory = Memory()
        # self.lock = threading.Lock() # lock to protect the shared memory

        self.accept_thread = threading.Thread(target=self.accept_connections)
        self.accept_thread.start()

    def handle_client(self,client_socket,client_address):
        print("Connection from {}".format(client_address))
        try:
            while True:
                message = client_socket.recv(1024).decode('utf-8')
                if not message:
                    break
                print("Received from client: {}".format(message))
                self.handle_message(client_socket,message)
                # client_socket.send("Message received".encode('utf-8'))
        finally:
            client_socket.close()
            print("Connection closed with {}".format(client_address))

    def accept_connections(self):
        while not self.shutdown_flag:
            try:
                client_socket, client_address = self.server_socket.accept()
                client_thread = threading.Thread(target=self.handle_client, args=(client_socket, client_address))
                client_thread.start()
            except socket.timeout:
                # Timeout reached, check shutdown_flag again
                continue

    def handle_message(self,client_socket,message):
        # DesiredHeading in deg
        if message.startswith("DesiredHeadingDeg"):
            with memory.lock: # get access to the shared memory
                memory.heading_d = sawtooth(float(message.split(":")[1])*3.14159/180.0) # convert deg to rad
                print("New desired heading is: {}".format(memory.heading_d))
                client_socket.send("Roger Roger".encode())
        elif message.startswith("DesiredHeadingRad"):
            with memory.lock: # get access to the shared memory
                memory.heading_d = sawtooth(float(message.split(":")[1]))
                print("New desired heading is: {}".format(memory.heading_d))
                client_socket.send("Roger Roger".encode())
        elif message.startswith("DesiredPosition"):
            with memory.lock:
                p = message.split(":")[1].split(",")
                memory.pos_d = np.array([[float(p[0]),float(p[1])]]).T
                print("New desired heading is: {}".format(memory.pos_d))
                client_socket.send("Roger Roger".encode())
        elif message.startswith("DesiredSpeed"):
            with memory.lock:
                vw = message.split(":")[1].split(",")
                memory.speed_d = float(vw[0])
                memory.rotation_speed_d = float(vw[1])
                print("New desired speed is: {} m/s and rotation speed is: {} rad/s".format(memory.speed_d,memory.rotation_speed_d))
                client_socket.send("Roger Roger".encode())
        elif message.startswith("ControlMode"):
            with memory.lock:
                control_mode = message.split(":")[1]
                # check if the control mode is valid
                if memory.control_mode not in ["Standby","Heading","Waypoint","Speed"]:
                    print("Invalid control mode")
                    return
                memory.control_mode = control_mode
                print("New control mode is: {}".format(memory.control_mode))
                memory.current_goal()
                client_socket.send("Roger Roger".encode())

        elif message.startswith("Heading"):
            with memory.lock:
                # send the current heading to the client in response
                print("Sending current heading: {}".format(memory.heading_g))
                client_socket.send(format(memory.heading_g).encode())
        elif message.startswith("MotorCmd"):
            with memory.lock:
                # send the current motor commands to the client in response
                print("Sending current motor commands: L{} R{}".format(memory.cmdl_g,memory.cmdr_g))
                client_socket.send("L{} R{}".format(memory.cmdl_g,memory.cmdr_g).encode())
        elif message.startswith("Position"):
            with memory.lock:
                # send the current local position to the client in response
                print("Sending current position: X{} Y{}".format(memory.pos_g[0,0],memory.pos_g[1,0]))
                client_socket.send("X{} Y{}".format(memory.pos_g[0,0],memory.pos_g[1,0]).encode())
        elif message.startswith("Attitude"):
            with memory.lock:
                # send the current attitude to the client in response
                print("Sending current attitude: Roll{} Pitch{} Yaw{}".format(memory.roll_g,memory.pitch_g,memory.heading_g))
                client_socket.send("Roll{} Pitch{} Yaw{}".format(memory.roll_g,memory.pitch_g,memory.heading_g).encode())
        else:
            print("Invalid message received: {}".format(message))
            client_socket.send("I don't understand !".encode())

if __name__ == "__main__":
    # Receive data from the client
    imu = imudv.Imu9IO()
    imu.load_calibration_parameters("compass/compass_parameters_5.json")

    DS = DdboatServer()

    # Main thread can perform other tasks
    try:
        while True:
            heading = imu.heading_simple(imu.cal_mag(imu.read_mag_raw()))
            with memory.lock:
                memory.heading_g = heading
                heading_d = memory.heading_d
            print("d_th {}, th {}".format(heading_d*180/3.14159,heading*180/3.14159))
            time.sleep(0.1)  # Replace this with any other tasks you want to perform
    except KeyboardInterrupt:
        print("Server shutting down.")
        shutdown_flag = True
    finally:
        DS.server_socket.close() # Close the server socket to unblock accept()
        DS.accept_thread.join() # Wait for the accept thread to finish
        print("Server shut down.")