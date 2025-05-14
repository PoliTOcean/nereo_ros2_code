import cv2
import time
from threading import Thread, Lock
import socket
import numpy as np
from queue import Queue
from PyQt6.QtGui import QImage
from PyQt6.QtCore import pyqtSignal, QObject

HOST_IP = '10.0.0.3'
PORT = 9999

class ImageReceiver(QObject):
    image_received = pyqtSignal(QImage)
    connection_status = pyqtSignal(bool, str)

    def __init__(self, host_ip: str = HOST_IP, port: int = PORT, fps: int = 15) -> None:
        super().__init__()
        self.fps = fps
        self.running = False
        self.retry_connect = True
        self.host_ip = host_ip
        self.port = port

        # Frame rate limiting
        self.frame_interval = 1.0 / fps
        self.last_frame_time = time.time()

        # Simplified queue with only latest frame
        self.frame_queue = Queue(maxsize=1)
        self.frame_lock = Lock()
        
        # Performance tracking
        self.received_frames = 0
        self.last_stat_time = time.time()
        self.last_handshake_time = 0
        
        # Fragment reassembly storage
        self.fragments = {}

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

        while not self.frame_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except:
                pass

        self.camera_thread.join(timeout=2.0)
        self.gui_thread.join(timeout=2.0)
        print("Client stopped!")

    def update_gui(self) -> None:
        while self.running:
            try:
                frame = self.frame_queue.get(timeout=0.1)
                if frame is not None:
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
        Run the UDP client and receive frames from server.
        """
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)  # 1MB buffer
            self.client_socket.settimeout(5)

            # Send handshake to the server
            self.last_handshake_time = time.time()
            self.client_socket.sendto(b"HELLO", (self.host_ip, self.port))
            print(f"Sent HELLO to server at {self.host_ip}:{self.port}")

            self.connection_status.emit(True, "UDP Client started")

            while self.running:
                # Resend handshake every 5 seconds
                current_time = time.time()
                if current_time - self.last_handshake_time > 5:
                    self.client_socket.sendto(b"HELLO", (self.host_ip, self.port))
                    self.last_handshake_time = current_time
                
                frame = self.receive_frame()
                if frame is not None:
                    # Always keep the latest frame, discard old ones
                    if not self.frame_queue.empty():
                        try:
                            self.frame_queue.get_nowait()
                        except:
                            pass
                    try:
                        self.frame_queue.put(frame, block=False)
                    except:
                        print("Failed to put frame in queue")
                    
                    # Track performance
                    self.received_frames += 1
                    if current_time - self.last_stat_time > 5:
                        fps = self.received_frames / (current_time - self.last_stat_time)
                        print(f"Receiving at {fps:.1f} FPS")
                        self.received_frames = 0
                        self.last_stat_time = current_time
                else:
                    time.sleep(0.01)  # Reduced sleep time for faster response

        except Exception as e:
            print(f"UDP run error: {e}")
            self.connection_status.emit(False, f"UDP Error: {e}")
        finally:
            self.stop()

    def receive_frame(self) -> np.ndarray | None:
        try:
            packet, _ = self.client_socket.recvfrom(65536)  # 64 KB buffer
            if not packet or len(packet) < 2:
                return None
            
            # First byte is the number of fragments
            num_fragments = packet[0]
            
            if num_fragments == 1:
                # Single packet frame
                nparr = np.frombuffer(packet[2:], np.uint8)
            else:
                # Multi-packet frame
                fragment_id = packet[1]
                
                # Create a new frame assembly if needed
                if fragment_id == 0:
                    self.fragments = {}
                
                # Store this fragment
                self.fragments[fragment_id] = packet[2:]
                
                # Check if we have all fragments
                if len(self.fragments) < num_fragments:
                    return None
                
                # Reassemble the frame
                reassembled = b''
                for i in range(num_fragments):
                    if i not in self.fragments:
                        print(f"Missing fragment {i}")
                        return None
                    reassembled += self.fragments[i]
                
                nparr = np.frombuffer(reassembled, np.uint8)
            
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            return frame

        except socket.timeout:
            # Only log timeout every few seconds to avoid spam
            if not hasattr(self, 'last_timeout_log') or time.time() - self.last_timeout_log > 5:
                print("UDP receive timeout")
                self.last_timeout_log = time.time()
            return None
        except Exception as e:
            print(f"Error receiving UDP frame: {e}")
            return None

    @staticmethod
    def frame_to_qimage(frame: np.ndarray) -> QImage | None:
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
