import cv2
import time
from threading import Thread, Lock
import socket
import struct
from queue import Queue
from PyQt6.QtGui import QImage
from PyQt6.QtCore import pyqtSignal, QObject
import numpy as np

HOST_IP = '127.0.0.1'
PORT = 9999

def recvall(sock: socket.socket, length: int) -> bytes | None:
    data = b''
    while len(data) < length:
        packet = sock.recv(min(length - len(data), 4096))
        if not packet:
            return None
        data += packet
    return data

class ImageReceiver(QObject):
    """
    Class to receive the image from the server and send it to the GUI.
    """
    image_received = pyqtSignal(QImage)
    connection_status = pyqtSignal(bool, str)

    def __init__(self, host_ip: str = HOST_IP, port: int = PORT, fps: int = 15) -> None:
        super().__init__()
        self.fps = fps
        self.running = False
        self.retry_connect = True
        self.host_ip = host_ip
        self.port = port
        self.payload_size = struct.calcsize("Q")
        self.max_retries = 25
        self.retry_delay = 0.5
        
        # Frame rate limiting
        self.frame_interval = 1.0 / fps
        self.last_frame_time = time.time()
        
        # Queue for thread-safe communication with smaller buffer
        self.frame_queue = Queue(maxsize=2)
        self.frame_lock = Lock()
        
        # Threads
        self.camera_thread = Thread(target=self.run)
        self.gui_thread = Thread(target=self.update_gui)
        self.camera_thread.daemon = True
        self.gui_thread.daemon = True

    def start(self) -> None:
        self.running = True
        self.camera_thread.start()
        self.gui_thread.start()

    def stop(self) -> None:
        self.running = False
        self.connection_status.emit(False, "Client stopped")
        # clear queue
        while not self.frame_queue.empty():
            try: self.frame_queue.get_nowait()
            except: pass
        # join threads
        self.camera_thread.join(timeout=2.0)
        self.gui_thread.join(timeout=2.0)
        print("Client stopped!")

    def update_gui(self) -> None:
        while self.running:
            try:
                frame = self.frame_queue.get(timeout=0.1)
                if frame is not None:
                    # frame rate limiting
                    now = time.time()
                    diff = now - self.last_frame_time
                    if diff < self.frame_interval:
                        time.sleep(self.frame_interval - diff)
                    qimg = self.frame_to_qimage(frame)
                    if qimg:
                        self.image_received.emit(qimg)
                        self.last_frame_time = time.time()
            except:
                continue

    def run(self) -> None:
        retries = 0
        while self.retry_connect and retries < self.max_retries:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                    sock.settimeout(5)  # only for connect
                    sock.connect((self.host_ip, self.port))
                    sock.settimeout(None)  # switch to blocking for recv()
                    print("Connected to server!")
                    self.connection_status.emit(True, "Connected to server")
                    retries = 0

                    while self.running:
                        frame = self.receive_frame(sock)
                        if frame is not None:
                            try:
                                self.frame_queue.put(frame, block=False)
                            except:
                                # drop old and retry
                                while not self.frame_queue.empty():
                                    try: self.frame_queue.get_nowait()
                                    except: pass
                                try:
                                    self.frame_queue.put(frame, block=False)
                                except:
                                    print("Queue full, frame dropped")
                        else:
                            print("Empty frame or disconnected")
                            break

            except socket.timeout:
                retries += 1
                print(f"Connection attempt {retries} timed out. Retrying...")
                self.connection_status.emit(False, f"Connection attempt {retries} failed")
                time.sleep(self.retry_delay)

            except Exception as e:
                retries += 1
                print(f"Connection error: {e}")
                self.connection_status.emit(False, f"Error: {e}")
                time.sleep(self.retry_delay)

        if retries >= self.max_retries:
            print("Max retries reached. Stopping client.")
            self.connection_status.emit(False, "Max retries reached")
            self.retry_connect = False

        self.stop()

    def receive_frame(self, sock: socket.socket) -> np.ndarray | None:
        try:
            packed_size = recvall(sock, self.payload_size)
            if not packed_size:
                return None

            # BIG-endian unpack
            msg_size = struct.unpack(">Q", packed_size)[0]
            frame_data = recvall(sock, msg_size)
            if frame_data is None:
                return None

            # JPEG decode
            nparr = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            return frame

        except Exception as e:
            print(f"Error receiving frame: {e}")
            return None

    @staticmethod
    def frame_to_qimage(frame: np.ndarray) -> QImage | None:
        try:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            return QImage(rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        except Exception as e:
            print(f"Error converting to QImage: {e}")
            return None
