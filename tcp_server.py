import socket
from imu9_driver_v2 import Imu9IO

# Server configuration
SERVER_HOST = "172.19.146.205"  # or you can use the IP address of this computer (e.g., '192.168.1.x')
SERVER_PORT = 5000  # Use any available port number

# Create the server socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((SERVER_HOST, SERVER_PORT))
server_socket.listen(1)  # Listen for 1 connection

print("Server listening on {}:{}".format(SERVER_HOST, SERVER_PORT))

# Accept a connection from a client
client_socket, client_address = server_socket.accept()
print("Connection from {}".format(client_address))

# Receive data from the client
imu = Imu9IO()
imu.load_calibration_parameters("compass/compass_parameters.json")
while True:
    message = client_socket.recv(1024).decode()
    if not message:
        break
    print("Received: {}".format(message))

    # if the message is like "d_th:XXX", get the value XXX of the desired heading
    if message.startswith("d_th:"):
        desired_heading = int(message.split(":")[1])
        print("The desired heading is: {}".format(desired_heading))

        # send the current heading to the client in response
        current_heading = imu.heading_simple(imu.correct_data(imu.read_mag_raw()))
        client_socket.send("Current heading: {}".format(current_heading).encode())
    else:
        client_socket.send("Message received".encode())  # Send response back to client

# Close the connection
client_socket.close()
server_socket.close()