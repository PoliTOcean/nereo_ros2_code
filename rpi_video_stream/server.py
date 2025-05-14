import cv2
import socket
import struct
import time
import threading

HOST_IP = '0.0.0.0'
PORT = 9999

class UDPCameraStreamer:
    def __init__(self, host_ip, port):
        self.host_ip = host_ip
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_address = None  # Will be set when the first client sends a request
        self.running = False

        # Optimization
        self.target_width = 320
        self.target_height = 240
        self.jpeg_quality = 50
        self.frame_interval = 1.0 / 15
        self.last_frame_time = time.time()

    def wait_for_client(self):
        print(f"Waiting for client on UDP port {self.port}...")
        while not self.client_address:
            try:
                data, addr = self.server_socket.recvfrom(1024)
                if data == b'HELLO':
                    self.client_address = addr
                    print(f"Client connected from {addr}")
            except Exception as e:
                print("Error waiting for client:", e)

    def start_streaming(self):
        self.running = True
        self.wait_for_client()

        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.target_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.target_height)
        cap.set(cv2.CAP_PROP_FPS, 15)

        if not cap.isOpened():
            print("Failed to open the camera.")
            return

        while self.running:
            current_time = time.time()
            if current_time - self.last_frame_time < self.frame_interval:
                time.sleep(0.01)
                continue

            ret, frame = cap.read()
            if not ret:
                continue

            frame = cv2.resize(frame, (self.target_width, self.target_height))
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            compressed_frame = buffer.tobytes()

            if len(compressed_frame) > 65000:
                print("Frame too large for UDP, skipping...")
                continue

            try:
                self.server_socket.sendto(compressed_frame, self.client_address)
                self.last_frame_time = current_time
            except Exception as e:
                print("Failed to send frame:", e)

        cap.release()

    def stop(self):
        self.running = False
        self.server_socket.close()

def main():
    streamer = UDPCameraStreamer(HOST_IP, PORT)
    try:
        streamer.start_streaming()
    except KeyboardInterrupt:
        print("Stopping stream...")
        streamer.stop()

if __name__ == "__main__":
    main()
