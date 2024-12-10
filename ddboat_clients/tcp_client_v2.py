import socket
import threading

class DdboatClient:
    def __init__(self):
        # Server details
        SERVER_HOST = "172.19.146.205"  # Replace with the server's IP address
        SERVER_PORT = 5000  # Same port as the server

        # Create the client socket
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.client_socket.connect((SERVER_HOST, SERVER_PORT))
        print(f"Connected to server at {SERVER_HOST}:{SERVER_PORT}")

# def ask_input(DC):
#     while True:
#         inp = input("Enter the desired heading in deg:")
#
#         # Send a message to the server
#         message = "DesiredHeading:" + inp
#         DC.client_socket.send(message.encode())
#
#         # Receive the server's response
#         response_ = DC.client_socket.recv(1024).decode()
#         print(f"Server response: {response_}")
#
#         # ask for the current heading
#         DC.client_socket.send("Heading".encode())
#
#         # Receive the server's response
#         response_ = DC.client_socket.recv(1024).decode()
#         print(f"Server response: {response_}")


if __name__ == "__main__":
    DC = DdboatClient()

    # create a threading that ask for a desired heading input
    # message_thread = threading.Thread(target=ask_input, args=(DC,))

    try:
        while True:
            inp = input("Enter the desired heading in deg:")

            # Send a message to the server
            message = "DesiredHeadingDeg:" + inp
            DC.client_socket.send(message.encode())

            # Receive the server's response
            response_ = DC.client_socket.recv(1024).decode()
            print(f"Server response: {response_}")

            # ask for the current heading
            print("Asking for the current heading")
            DC.client_socket.send("Heading".encode())

            # Receive the server's response
            response_ = DC.client_socket.recv(1024).decode()
            print(f"Server response: {response_}")
    except KeyboardInterrupt:
        print("Shutting down the client...")
    finally:
        # Close the connection
        DC.client_socket.close()