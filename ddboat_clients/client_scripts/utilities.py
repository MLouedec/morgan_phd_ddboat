import socket
import numpy as np

def sawtooth(angle):
        return (angle+np.pi)%(2*np.pi)-np.pi 

def connectSocket():
    # Server details
    SERVER_HOST = "172.19.146.205"  # Replace with the server's IP address
    SERVER_PORT = 5000  # Same port as the server

    # Create the client socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    client_socket.connect((SERVER_HOST, SERVER_PORT))
    print(f"Connected to server at {SERVER_HOST}:{SERVER_PORT}")

    return client_socket