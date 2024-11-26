import socket
from imu9_driver_v2 import Imu9IO
import threading
import time

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

        self.desired_heading = 0. # rad
        self.current_heading = 0. # rad

        self.cmdL = 0 # current left motor command
        self.cmdR = 0 # current right motor command

        # # Accept a connection from a client
        # client_socket, client_address = self.server_socket.accept()
        # print("Connection from {}".format(client_address))

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
        if message.startswith("DesiredHeading:"):
            self.desired_heading = float(message.split(":")[1])*3.14159/180.0 # convert deg to rad
            print("New desired heading is: {}".format(self.desired_heading))
            client_socket.send("Roger Roger".encode())
        elif message.startswith("Heading"):
            # send the current heading to the client in response
            client_socket.send(format(self.current_heading).encode())
        elif message.startswith("MotorCmd"):
            # send the current motor commands to the client in response
            client_socket.send("L{} R{}".format(self.cmdL,self.cmdR).encode())
        else:
            print("Invalid message received: {}".format(message))
            client_socket.send("I don't understand !".encode())

if __name__ == "__main__":
    # Receive data from the client
    imu = Imu9IO()
    imu.load_calibration_parameters("compass/compass_parameters_5.json")

    DS = DdboatServer()

    # Main thread can perform other tasks
    try:
        while True:
            DS.current_heading = imu.heading_simple(imu.cal_mag(imu.read_mag_raw()))
            print("d_th {}, th {}".format(DS.desired_heading*180/3.14159,DS.current_heading*180/3.14159))
            time.sleep(0.1)  # Replace this with any other tasks you want to perform
    except KeyboardInterrupt:
        print("Server shutting down.")
        shutdown_flag = True
    finally:
        DS.server_socket.close() # Close the server socket to unblock accept()
        DS.accept_thread.join() # Wait for the accept thread to finish
        print("Server shut down.")