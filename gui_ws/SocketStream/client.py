import cv2
import socket
import pickle
import struct

# Socket Setup
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_ip = '127.0.0.1'  # Replace with the server's IP address
port = 9999

# Connect to the server
client_socket.connect((host_ip, port))

data = b""
payload_size = struct.calcsize("Q")

while True:
    # Keep receiving data until we get enough for a frame
    while len(data) < payload_size:
        packet = client_socket.recv(4 * 1024)  # 4K buffer size
        if not packet:
            break
        data += packet

    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack("Q", packed_msg_size)[0]

    while len(data) < msg_size:
        data += client_socket.recv(4 * 1024)

    frame_data = data[:msg_size]
    data = data[msg_size:]

    # Deserialize the frame
    frame = pickle.loads(frame_data)

    # Display the frame
    cv2.imshow("Client - Receiving", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
client_socket.close()
cv2.destroyAllWindows()

