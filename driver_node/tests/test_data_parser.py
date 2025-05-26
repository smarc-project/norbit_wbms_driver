import logging
import socket
import struct
import time

class TestDataParser():
    """
    Test []_parser.py by setting up a mock TCP server at local host
    and piping example data to designated data port.
    """

    def __init__(self, port, data_filepath):
        self.logger = logging.getLogger(__name__)
        self.ip = '127.0.0.1'
        self.port = port
        self.tcp_connection = self._setup_tcp_server()
        
        self.data_filepath = data_filepath
        self._load_data()

    def _load_data(self):
        with open(self.data_filepath, 'br') as f:
            data = f.read()
        self.data = data
        self.msg_size = struct.unpack('<I', data[8:12])[0] #TODO: usee []Parser instead!
        self.total_size = len(data)
        self.num_msg = self.total_size // self.msg_size
        assert self.num_msg * self.msg_size == self.total_size, f"Total size {self.total_size} is not a multiple of message size {self.msg_size}"

    def _setup_tcp_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.ip, self.port))
            s.listen(5)
            conn, addr = s.accept()
            self.logger.info(f'Accepted connection from {addr}')
            return conn

    def send_one_chunk(self, chunk_id):
        if chunk_id < 0 or chunk_id >= self.num_msg:
            raise ValueError(f"Chunk ID {chunk_id} is out of range. Must be between 0 and {self.num_msg - 1}.")
        start = chunk_id * self.msg_size
        end = start + self.msg_size
        self.logger.info(f'Sending chunk {chunk_id} from bytes {start} to {end}')
        self.tcp_connection.sendall(self.data[start:end])
        

    def send_all_data(self, delay=1):
            for i in range(0, self.total_size, self.msg_size):
                print(f'sending bytes {i} to {i+self.msg_size}')
                self.tcp_connection.sendall(self.data[i:i+self.msg_size])
                time.sleep(delay)