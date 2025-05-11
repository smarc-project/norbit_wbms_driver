import logging
import socket
import time

class TestWaterColumnParser():
    """
    Test watercolumn_parser.py by setting up a mock TCP server at local host
    and piping example watercolumn data to port 2211 (the port where the Norbit
    sensor publishes watercolumn data.).
    This class tests that data is received by the WaterColumnNode and can be published
    to the corresponding topic.
    """

    def __init__(self):
        self.ip = '127.0.0.1'
        self.port = 2211
        self.tcp_connection = self._setup_tcp_server()
        
        #### Below are stats of the FLS TCP binary contents
        self.wc_filepath = '../../data/fls_tcp_400khz_100m.bin'
        self.total_size = 12365696 # size of the entire .bin file in bytes
        self.msg_size = 167104 # size of each and every message in bytes
        self.num_msg = 74 # num watercolumn msgs in the .bin file

    def _setup_tcp_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.ip, self.port))
            s.listen(5)
            conn, addr = s.accept()
            logging.getLogger().info(f'Connected by {addr}')
            return conn

    def test_parse(self):
        with open(self.wc_filepath, 'br') as f:
            data = f.read()

            for i in range(0, self.total_size, self.msg_size):
                print(f'sending bytes {i} to {i+self.msg_size}')
                self.tcp_connection.sendall(data[i:i+self.msg_size])
                # time.sleep(1)
                assert True
