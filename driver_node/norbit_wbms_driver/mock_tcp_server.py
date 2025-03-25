"""
A mock server to test WBMS driver TCP connections.
It pipes all data straight back to the client.
"""
import socket

host = '127.0.0.1'
port = 2209

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((host, port))
    s.listen(5)

    conn, addr = s.accept()
    print(f'Connected by {addr}')
    while True:
        data = conn.recv(1024)
        conn.sendall(data)
        # update connection
        conn, addr = s.accept()
        print(f'Connected by {addr}')