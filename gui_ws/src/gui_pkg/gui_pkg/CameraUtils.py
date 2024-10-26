import cv2
import time
from threading import Thread
import socket
import struct
import pickle
from PyQt6.QtGui import QImage
from PyQt6.QtCore import pyqtSignal, QObject

class ImageReceiver(QObject):
    image_received = pyqtSignal(QImage)
    connection_status = pyqtSignal(bool, str)

    def __init__(self, host_ip='localhost', port=9999, fps=30):
        super().__init__()
        self.fps = fps
        self.running = False
        self.host_ip = host_ip
        self.port = port
        self.payload_size = struct.calcsize("Q")
        self.max_retries = 25
        self.retry_delay = 0.5  # 0.1 seconds (one tenth)

    def start(self):
        self.running = True
        Thread(target=self.run).start()

    def stop(self):
        self.running = False
        self.connection_status.emit(False, "Client stopped")
        print("Client stopped!")

    def run(self):
        retries = 0
        while self.running and retries < self.max_retries:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as self.client_socket:
                    self.client_socket.settimeout(5)  # 5 seconds timeout for connection
                    self.client_socket.connect((self.host_ip, self.port))
                    print("Connected to server!")
                    self.connection_status.emit(True, "Connected to server")
                    retries = 0  # Reset retry counter on successful connection

                    while self.running:
                        frame = self.receive_frame()
                        if frame is not None:
                            q_image = self.frame_to_qimage(frame)
                            self.image_received.emit(q_image)
                        #time.sleep(1/self.fps)

            except socket.timeout:
                retries += 1
                print(f"Connection attempt {retries} timed out. Retrying...")
                self.connection_status.emit(False, f"Connection attempt {retries} failed")
                time.sleep(self.retry_delay)

            except Exception as e:
                retries += 1
                print(f"An error occurred: {e}")
                self.connection_status.emit(False, f"Error: {e}")
                time.sleep(self.retry_delay)

        if retries >= self.max_retries:
            print("Max retries reached. Stopping client.")
            self.connection_status.emit(False, "Max retries reached")

        self.stop()

    def receive_frame(self):
        data = b""
        print("Receiving frame...")
        while len(data) < self.payload_size:
            packet = self.client_socket.recv(4*1024)
            if not packet:
                return None
            data += packet
        packed_msg_size = data[:self.payload_size]
        data = data[self.payload_size:]
        msg_size = struct.unpack("Q", packed_msg_size)[0]
        while len(data) < msg_size:
            data += self.client_socket.recv(4*1024)
        frame_data = data[:msg_size]
        print("Frame received!")
        return pickle.loads(frame_data)

    @staticmethod
    def frame_to_qimage(frame):
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        return QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
