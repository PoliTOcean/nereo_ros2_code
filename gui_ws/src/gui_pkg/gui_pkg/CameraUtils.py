import cv2
import time
from threading import Thread, Lock
import socket
import struct
import pickle
from queue import Queue
from PyQt6.QtGui import QImage
from PyQt6.QtCore import pyqtSignal, QObject
import numpy as np

# Address for Raspberry: host_ip = 10.0.0.3, port = 8080
# Address for PC: host_ip = 0.0.0.0, port = 9999

"""
HOST_IP = '10.0.0.3'
PORT = 8080
"""

HOST_IP = '0.0.0.0'
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
        self.frame_queue = Queue(maxsize=2)  # Reduced queue size
        self.frame_lock = Lock()
        
        # Create threads
        self.camera_thread = Thread(target=self.run)
        self.gui_thread = Thread(target=self.update_gui)
        
        # Set both threads as daemon threads
        self.camera_thread.daemon = True
        self.gui_thread.daemon = True

    def start(self) -> None:
        """
        Function to start the camera thread and the GUI thread.
        """
        self.running = True
        self.camera_thread.start()
        self.gui_thread.start()

    def stop(self) -> None:
        """
        Function to stop the camera thread and the GUI thread.
        """
        self.running = False
        self.connection_status.emit(False, "Client stopped")
        
        # Clear the queue
        while not self.frame_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except:
                pass
                
        # Wait for threads to finish with timeout
        self.camera_thread.join(timeout=2.0)
        self.gui_thread.join(timeout=2.0)
        print("Client stopped!")

    def update_gui(self) -> None:
        """
        Separate thread for updating the GUI with camera frames
        """
        while self.running:
            try:
                # Get frame from queue with timeout
                frame = self.frame_queue.get(timeout=0.1)
                if frame is not None:
                    # Frame rate limiting
                    current_time = time.time()
                    elapsed = current_time - self.last_frame_time
                    if elapsed < self.frame_interval:
                        time.sleep(self.frame_interval - elapsed)
                    
                    try:
                        q_image = self.frame_to_qimage(frame)
                        if q_image is not None:
                            self.image_received.emit(q_image)
                            self.last_frame_time = time.time()
                    except Exception as e:
                        print(f"Error converting frame to QImage: {e}")
                        continue
            except:
                continue

    def run(self) -> None:
        """
        Function to run the camera thread and put the frame in the queue.
        """
        retries = 0
        while self.retry_connect and retries < self.max_retries:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as self.client_socket:
                    self.client_socket.settimeout(5)  # 5 seconds timeout for connection
                    self.client_socket.connect((self.host_ip, self.port))
                    print("Connected to server!")
                    self.connection_status.emit(True, "Connected to server")
                    retries = 0

                    while self.running:
                        try:
                            frame = self.receive_frame()
                            if frame is not None:
                                # Put frame in queue, drop if full
                                try:
                                    self.frame_queue.put(frame, block=False)
                                except:
                                    # If queue is full, clear it and try again
                                    while not self.frame_queue.empty():
                                        try:
                                            self.frame_queue.get_nowait()
                                        except:
                                            pass
                                    try:
                                        self.frame_queue.put(frame, block=False)
                                    except:
                                        print("Failed to put frame in queue")
                            else:
                                print("Received empty frame")
                                break
                        except Exception as e:
                            print(f"Frame reception error: {e}")
                            self.connection_status.emit(False, f"Frame error: {e}")
                            break

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

            finally:
                # Ensure socket is closed and we're ready for next connection attempt
                try:
                    self.client_socket.close()
                except:
                    pass

        if retries >= self.max_retries:
            print("Max retries reached. Stopping client.")
            self.connection_status.emit(False, "Max retries reached")
            self.retry_connect = False  # Stop trying to connect
            print("Stopped trying to connect.")

        self.stop()  # Ensure proper cleanup when done

    def receive_frame(self) -> np.ndarray | None: #TODO: Check the correct type
        """
        Function to receive the frame from the server.
        """
        try:
            packed_msg_size = recvall(self.client_socket, self.payload_size)
            if not packed_msg_size:
                print("Failed to receive frame size")
                return None

            msg_size = struct.unpack("Q", packed_msg_size)[0]

            frame_data = recvall(self.client_socket, msg_size)
            if frame_data is None:
                print("Failed to receive complete frame data")
                return None

            # Decode JPEG compressed frame
            nparr = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if frame is None:
                print("Warning: Failed to decode JPEG frame")
                return None

            return frame
            
        except Exception as e:
            print(f"Error receiving frame: {e}")
            return None

    @staticmethod
    def frame_to_qimage(frame: np.ndarray) -> QImage | None: #TODO: Check the correct type
        """
        Function to convert the frame to a QImage.
        """
        try:
            if frame is None:
                return None
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            return QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        except Exception as e:
            print(f"Error converting frame to QImage: {e}")
            return None
