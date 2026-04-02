import cv2
import socket
import pickle
import struct

cap = cv2.VideoCapture(0)

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(("192.168.5.19", 9999))  # 改成你电脑IP

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 降分辨率（强烈建议）
    frame = cv2.resize(frame, (320, 240))

    data = pickle.dumps(frame)
    message = struct.pack("Q", len(data)) + data
    client_socket.sendall(message)

cap.release()
client_socket.close()
