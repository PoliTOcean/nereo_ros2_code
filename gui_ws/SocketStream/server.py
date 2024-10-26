import cv2
import socket
import pickle
import struct
import time

# Socket Setup
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_ip = '0.0.0.0'  # Listen on all available interfaces
port = 9999
socket_address = (host_ip, port)

class SocketCommunication():
    def __init__(self, server_socket, host_ip, port):
        self.server_socket = server_socket
        self.client_socket = None
        self.socket_address = (host_ip, port)
        self.connection = False

        self.timeout = 1.0
        self.max_tentatives = 5

        self.cap = None

    def start_socket(self):
        self.server_socket.bind(self.socket_address)
        self.server_socket.listen(5)
        print(f"Listening at: {self.socket_address}")
        
        # Accept client connection
        client_socket, addr = self.server_socket.accept()
        print(f'Connection from: {addr}')

        self.client_socket = client_socket
        
        if self.client_socket:
            self.connection = True

    def start_transmitting(self):
        self.cap = cv2.VideoCapture(0)

        while self.cap.isOpened():
            try:
                ret, frame = self.cap.read()
                if not ret:
                    break

                # Serialize frame (encode the image)
                data = pickle.dumps(frame)
                # Send message length first (so the client knows how much to read)
                message = struct.pack("Q", len(data)) + data

                # Send the serialized frame over the socket
                self.client_socket.sendall(message)
            except:
                if self.max_tentatives == 0:
                    self.clean_all()
                    break

                self.max_tentatives -= 1
                self.try_reconnect()

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def try_reconnect(self):
        print("Trying to reconnect...")
        time.sleep(self.timeout)
        self.start_socket()

    def clean_all(self):
        self.cap.release()
        self.client_socket.close()
        self.server_socket.close()
        cv2.destroyAllWindows()


def main():
    socket_communication = SocketCommunication(server_socket, host_ip, port)
    socket_communication.start_socket()
    
    if socket_communication.connection:
        socket_communication.start_transmitting()

    socket_communication.clean_all()


if __name__ == "__main__":
    main()
