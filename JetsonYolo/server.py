# echo-server.py

import socket
import time

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    while (True):
        print("Waiting for signal")
        s.listen()
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            rec_data = conn.recv(1024)
            print(rec_data)

            '''
            Operation
            '''
            print("Operating")
            time.sleep(10)
            '''
            Operation
            '''


            send_data = b"Down"
            conn.sendall(send_data)


