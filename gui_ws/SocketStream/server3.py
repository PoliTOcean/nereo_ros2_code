import cv2
import socket
import pickle
import struct
import time
import threading

class SocketCommunication:
    def __init__(self, host_ip, port):
        self.host_ip = host_ip
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.socket_address = (host_ip, port)
        self.connection = False
        self.cap = None
        self.running = False

    def start_socket(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(self.socket_address)
        self.server_socket.listen(5)
        print(f"Listening at: {self.socket_address}")

    def accept_connection(self):
        while self.running:
            try:
                self.client_socket, addr = self.server_socket.accept()
                print(f'Connection from: {addr}')
                self.client_socket.settimeout(5)  # Set timeout for the client socket
                self.connection = True
                return True
            except socket.timeout:
                pass
        return False

    def start_transmitting(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Failed to open video capture!")
            return
        self.running = True

        frame_counter = 0  # Count frames to track any sudden stops

        while self.running:
            if not self.connection:
                print("Waiting for connection...")
                if not self.accept_connection():
                    continue

            try:
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to capture frame")
                    break

                # Resize the frame to reduce the amount of data sent
                frame = cv2.resize(frame, (640, 480))

                # Compress frame using JPEG to reduce size
                result, frame = cv2.imencode('.jpg', frame)
                if not result:
                    print("Failed to encode frame")
                    break

                data = pickle.dumps(frame)
                message = struct.pack("Q", len(data)) + data

                self.client_socket.sendall(message)
                frame_counter += 1
                print(f"Sent frame {frame_counter}")

                # Introduce a small delay between frames to avoid overwhelming the socket
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

