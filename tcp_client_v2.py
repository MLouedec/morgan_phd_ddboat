import socket

# Server details
SERVER_HOST = "172.19.146.205"  # Replace with the server's IP address
SERVER_PORT = 5000  # Same port as the server

# Create the client socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

client_socket.connect((SERVER_HOST, SERVER_PORT))
print(f"Connected to server at {SERVER_HOST}:{SERVER_PORT}")

while True:
    inp = input("Enter the desired heading in deg:")


    # Send a message to the server
    message = "d_th:" + inp
    client_socket.send(message.encode())

    # Receive the server's response
    response = client_socket.recv(1024).decode()
    print(f"Server response: {response}")

# Close the connection
client_socket.close()