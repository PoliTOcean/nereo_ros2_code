import cv2
import socket
import pickle
import struct
import time
import os
import threading

# Wait time for the client to connect
TIMEOUT = 1
MAX_TENTATIVES = 10

class SocketCommunication:
    def __init__(self, host_ip, port):
        self.host_ip = host_ip
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.socket_address = (host_ip, port)
        self.connection = False
        self.timeout = 5
        self.cap = None
        self.running = False
        self.retry = 0
        self.reconnect_event = threading.Event()

    def start_socket(self):
        
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.server_socket.bind(self.socket_address)
        except socket.error as e:
            time.sleep(1)
            print(f"Error: {e}\n")
            # execute bash command to free the port
            if (self.retry < MAX_TENTATIVES):
                print("Freeing the port...")
                os.system(f"sudo fuser -k {self.port}/tcp")
                self.retry += 1
                self.start_socket()
            else:
                print(f"Failed to start the server. Max tentatives reached: {self.retry}")
                self.clean_all()
                exit(1)
        self.server_socket.listen(5)
        print(f"Listening at: {self.socket_address}")

    def accept_connection(self):
        if self.server_socket is None:
            print("Server not started\nAttempting to restart the server\n")
            self.start_socket()

        while self.running:
            try:
                self.client_socket, addr = self.server_socket.accept()
                print(f'Connection from: {addr}')
                self.client_socket.settimeout(self.timeout)
                self.connection = True
                self.reconnect_event.clear()
                return True
            except socket.timeout:
                pass
        return False

    def start_transmitting(self):
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            print("Failed to open the camera")
            self.clean_all()

        self.running = True

        while self.running:
            if not self.connection:
                if not self.accept_connection():
                    continue

            try:
                ret, frame = self.cap.read()
                if not ret:
                    raise Exception("Failed to capture frame")

                data = pickle.dumps(frame)
                message = struct.pack("Q", len(data)) + data
                self.client_socket.sendall(message)
                print("Frame sent")

                time.sleep(0.03)

            except (socket.timeout, socket.error, BrokenPipeError, Exception) as e:
                print(f"Error: {e}")
                self.handle_disconnect()

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.clean_all()

    def handle_disconnect(self):
        print("Connection lost. Attempting to reconnect...")
        self.connection = False
        if self.client_socket:
            self.client_socket.close()
        self.reconnect_event.set()

        while not self.connection and self.running:
            print("Trying to reconnect...")
            self.accept_connection()
            time.sleep(1) # To avoid too tight loops

    def clean_all(self):
        self.running = False
        if self.cap:
            self.cap.release()
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()
        cv2.destroyAllWindows()

    def stop(self):
        print("Stopping communication...")
        self.running = False
        self.reconnect_event.set()

def main():
    host_ip = '0.0.0.0'
    port = 9999
    socket_communication = SocketCommunication(host_ip, port)
    socket_communication.start_socket()

    # Start the communication in a separate thread
    comm_thread = threading.Thread(target=socket_communication.start_transmitting)
    comm_thread.start()

    # Wait for user input to stop the communication
    input("Press Enter to stop the communication...\n")
    socket_communication.stop()

    # Wait for the communication thread to finish
    comm_thread.join()

if __name__ == "__main__":
    main()
