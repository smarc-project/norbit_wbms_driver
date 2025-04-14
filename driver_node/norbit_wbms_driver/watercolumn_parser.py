#!/usr/bin/env python3
"""
norbit water column data parser.
"""
import struct
import socket
import numpy as np
import rclpy
from rclpy.node import Node
import time

from sensor_msgs.msg import Image
# from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from norbit_wbms_interfaces.msg import WaterColumn

__author__ = "Aldo Teran"
__author_email = "aldot@kth.se"
__license__ = "MIT"
__status__ = "Development"

SOCKET_TIMEOUT = 5


class WCParser:
    """
    Class for handling the raw logic of parsing a raw data packet.

    Based on Sec. 8.2 of the norbit's TN-180196 document.
    """
    def __init__(self):
        # Map from dtype to corresponding byte size for stepping and format
        # character for parsing. Keys correspond to the indices in `parse_dtype`.
        self.dtype_map = {
            0: ('B', 1),     # uint8
            1: ('b', 1),     # int8
            2: ('H', 2),     # uint16
            3: ('h', 2),     # int16
            4: ('I', 4),     # uint32
            5: ('i', 4),     # int32
            6: ('Q', 8),     # uint64
            7: ('q', 8),     # int64
            0x15: ('f', 4),  # float32
            0x17: ('d', 8),  # float64
        }


    def parse_preamble(self, msg):
        '''This should always be 0xDEADBEEF.'''
        return struct.unpack('<I', msg[0:4])[0]


    def parse_type(self, msg):
        '''This corresponds to the type of data being returned.'''
        return struct.unpack('<I', msg[4:8])[0]


    def parse_size(self, msg):
        '''Returns the size of the entire package in bytes.'''
        return struct.unpack('<I', msg[8:12])[0]


    def parse_sound_velocity(self, msg):
        '''Returns the sound velocity in m/s.'''
        return struct.unpack('<f', msg[24:28])[0]


    def parse_sample_rate(self, msg):
        '''Returns the sample rate in Hz.'''
        return struct.unpack('<f', msg[28:32])[0]


    def parse_num_beams(self, msg):
        '''Returns the number of available beams.'''
        return struct.unpack('<I', msg[32:36])[0]


    def parse_num_samples(self, msg):
        '''Returns the number samples in each beam.'''
        return struct.unpack('<I', msg[36:40])[0]


    def parse_timestamp(self, msg):
        '''Returns the ping's Unix timestamp at TX.'''
        return struct.unpack('<d', msg[40:48])[0]


    def parse_dtype(self, msg):
        '''Returns the underlying type of the data.

        The dtype options are:
            0:uint8
            1:int8
            2:uint16
            3:int16
            4:uint32
            5:int32
            6:uint64
            7:int64
            0x15:float32
            0x17:float64
        '''
        return struct.unpack('<I', msg[48:52])[0]

    def parse_first_sample_num(self, msg):
        '''Returns the sample number of first sample in data.'''
        return struct.unpack('<i', msg[52:56])[0]
    

    def parse_gain(self, msg):
        '''Returns the total processing gain.'''
        return struct.unpack('<f', msg[56:60])[0]
    

    def parse_swath_dir(self, msg):
        '''Returns the swath center direction in radians.'''
        return struct.unpack('<f', msg[64:68])[0]
    

    def parse_swath_open(self, msg):
        '''Returns the swath opening angle in radians.'''
        return struct.unpack('<f', msg[68:72])[0]
    
    
    def parce_tx_freq(self, msg):
        '''Returns the transmission frequency in kHz.'''
        return struct.unpack('<f', msg[72:76])[0]


    def parse_tx_bw(self, msg):
        '''Returns the transmission bandwidth in kHz.'''
        return struct.unpack('<f', msg[76:80])[0]
    

    def parse_tx_len(self, msg):
        '''Returns the transmission pulse length in sec.'''
        return struct.unpack('<f', msg[80:84])[0]
    

    def parse_tx_amp(self, msg):
        '''Returns the transmission pulse amplitude.'''
        return struct.unpack('<I', msg[84:88])[0]
    
    
    def parse_ping_rate(self, msg):
        '''Returns the ping rate in Hz.'''
        return struct.unpack('<f', msg[100:104])[0]
    

    def parse_ping_num(self, msg):
        '''Returns sequence number of ping.'''
        return struct.unpack('<I', msg[108:112])[0]
    

    def parse_time_net(self, msg):
        '''Returns the Timestamp unix time as fract (send on network time).'''
        return struct.unpack('<d', msg[112:120])[0]
    

    def parse_beams(self, msg):
        '''Returns the number of beams in beamformer before decimation.'''
        return struct.unpack('<I', msg[120:124])[0]
    

    def vga_t1(self, msg):
        '''Returns the sample number for first vga value.'''
        return struct.unpack('<i', msg[124:128])[0]
    

    def vga_g1(self, msg):
        '''Returns the gain for first vga value in dB.'''
        return struct.unpack('<f', msg[128:132])[0]
    

    def vga_t2(self, msg):
        '''Returns the sample number for second vga value.'''
        return struct.unpack('<i', msg[132:136])[0]
    

    def vga_g2(self, msg):
        '''Returns the gain for second vga value in dB.'''
        return struct.unpack('<f', msg[136:140])[0]
    
    
    def parse_tx_angle(self, msg):
        '''Returns the Tx elevation steering angle in radians.'''
        return struct.unpack('<f', msg[144:148])[0]
    

    def parse_tx_voltage(self, msg):
        '''Returns the peak voltage signal over ceramics, NaN for sonars without measurement.'''
        return struct.unpack('<f', msg[148:152])[0]
    

    def parse_beam_dist_mode(self, msg):
        '''Returns the beam distance mode.
        
        The modes are:
            1: 512EA
            2: 256EA
            3: 256ED
            4: 512EAx
            5. 256EAx
            
        '''
        return struct.unpack('<B', msg[152:153])[0]
    
    def parse_sonar_mode(self, msg):
        '''Returns the sonar mode.
        
        The modes are:
            0: FLS
            1: Bathy
            2: Bathy amp
            3: TBD
            4: bf passthrough
            5: Bathy edge enhance
            6: Super res bathy

        '''
        return struct.unpack('<B', msg[153:154])[0]
    

    def parse_gate_tilt(self, msg):
        '''Returns the gate tilt in radians, for depth gates.'''
        return struct.unpack('<f', msg[156:160])[0]

    def parse_pixel_data(self, msg):
        '''Returns the MxN pixel data array.

        M: number of samples in each beam
        N: number of beams
        '''
        M = self.parse_num_samples(msg)
        N = self.parse_num_beams(msg)

        # The step size in bytes depends on the dtype representation.
        dtype = self.parse_dtype(msg)
        step_char,step_size = self.dtype_map[dtype]
        parse_str = '<{}'.format(step_char)

        print("M={} N={}, num pixels={}".format(M,N,M*N))
        pixel_values = []
        for i in range(M * N):
            offset = 192 + i * step_size
            pixel_values.append(
                struct.unpack(parse_str, msg[offset:offset+step_size])[0])
        return pixel_values


    def parse_directions(self, msg):
        '''Returns the beam directions in radians.'''
        N = self.parse_num_beams(msg)
        M = self.parse_num_samples(msg)
        step_char,step_size = self.dtype_map[self.parse_dtype(msg)]

        fixed_offset = 192 + M*N*step_size

        directions = []
        for i in range(N):
            offset = fixed_offset + 4 * i
            directions.append(
                struct.unpack('<f', msg[offset:offset+4])[0])
        return directions



class WaterColumnNode(Node):
    """
    Class to handle the HEX parsing from an norbit sonar's data stream.
    Note that this node blocks until the TCP connection is established.
    """

    BUFFER_SIZE_BYTES = 512000

    def __init__(self):
        super().__init__('wc_parser')
        self.get_logger().info("Starting the WBMS water column parser node...")
        
        self.sonar_ip = '127.0.0.1' #TODO: modify in launch file
        self.watercolumn_port = 2211

        self.tcp_retry_every = 5 # seconds
        self.tcp_socket = self.connect_to_sonar()


        # Build our parser.
        self.p = WCParser()

        # Create the buffer where we'll store the data being streamed in.
        self.data_buffer = b''

        self.watercolumn_pub = self.create_publisher(WaterColumn, 'watercolumn', 1)
        self.watercolumn_pub_timer = self.create_timer(1, self.parse_and_publish)

    def connect_to_sonar(self):
        """
        Connect to the sonar's IP and port and return the TCP socket.
        Retry until connection is established.
        """
        connected = False

        tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_socket.settimeout(SOCKET_TIMEOUT)
        tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        while not connected:
            try:
                tcp_socket.connect((self.sonar_ip, self.watercolumn_port))
                self.get_logger().info(f"Connected to sonar at {self.sonar_ip}:{self.watercolumn_port}")
                connected = True
                return tcp_socket
            except Exception as e:
                self.get_logger().error(f"Error connecting to sonar at {self.sonar_ip}:{self.watercolumn_port}")
                self.get_logger().error(str(e))
                self.get_logger().info(f"Trying again in {self.tcp_retry_every} seconds...")
                time.sleep(self.tcp_retry_every)


    def parse_and_publish(self):
        """
        Parse and publish the data recived from the TCP socket.
        #TODO: Fix bug: use data buffer correctly
        # Cuurrently: if the TCP connection sends data faster than the
        # parser can process it, the data will be lost, and the remaining
        # partial messages will not pass the 0xDEADBEEF check.
        """
        try:
            # Fetch the initial chunk.
            data, addr = self.tcp_socket.recvfrom(self.BUFFER_SIZE_BYTES)
            # Quick check of the deadbeef.
            if self.p.parse_preamble(data) != 0xDEADBEEF:
                self.get_logger().error("Message did not pass the deadbeef check!")
                return

            self.data_buffer += data

            # Get the expected size of the packet.
            M = self.p.parse_num_samples(data)
            N = self.p.parse_num_beams(data)
            dtype = self.p.parse_dtype(data)
            _,step_size = self.p.dtype_map[dtype]
            expected_size_bytes = 192 + M*N*step_size + 4*N
            real_size = self.p.parse_size(data)

            # Keep looping until the full message is completely received.
            while len(self.data_buffer) < expected_size_bytes:
                try:
                    data, addr = self.tcp_socket.recvfrom(self.BUFFER_SIZE_BYTES)
                    self.data_buffer += data
                except:
                    print("something went wrong in the msg reconstruction loop")

        except socket.timeout:
            self.get_logger().error("Watercolumn interface socket timed out, verify connection.")
            return

        print("Final size of the concat msg={}".format(len(self.data_buffer)))

        # build the WaterColumn msg
        msg = WaterColumn()
        msg.preamble = self.p.parse_preamble(self.data_buffer)
        msg.type = self.p.parse_type(self.data_buffer)
        msg.size = self.p.parse_size(self.data_buffer)
        msg.snd_velocity = self.p.parse_sound_velocity(self.data_buffer)        
        msg.sample_rate = self.p.parse_sample_rate(self.data_buffer)
        msg.num_beams = self.p.parse_num_beams(self.data_buffer)
        msg.num_samples = self.p.parse_num_samples(self.data_buffer)
        msg.time = self.p.parse_timestamp(self.data_buffer)
        msg.dtype = self.p.parse_dtype(self.data_buffer)
        msg.t0 = self.p.parse_first_sample_num(self.data_buffer)
        msg.gain = self.p.parse_gain(self.data_buffer)
        msg.swath_dir = self.p.parse_swath_dir(self.data_buffer)
        msg.swath_open = self.p.parse_swath_open(self.data_buffer)
        msg.tx_freq = self.p.parce_tx_freq(self.data_buffer)
        msg.tx_bw = self.p.parse_tx_bw(self.data_buffer)
        msg.tx_len = self.p.parse_tx_len(self.data_buffer)
        msg.tx_amp = self.p.parse_tx_amp(self.data_buffer)
        msg.ping_rate = self.p.parse_ping_rate(self.data_buffer)
        msg.ping_number = self.p.parse_ping_num(self.data_buffer)
        msg.time_net = self.p.parse_time_net(self.data_buffer)
        msg.beams = self.p.parse_beams(self.data_buffer)
        msg.vga_t1 = self.p.vga_t1(self.data_buffer)
        msg.vga_g1 = self.p.vga_g1(self.data_buffer)
        msg.vga_t2 = self.p.vga_t2(self.data_buffer)
        msg.vga_g2 = self.p.vga_g2(self.data_buffer)
        msg.tx_angle = self.p.parse_tx_angle(self.data_buffer)
        msg.tx_voltage = self.p.parse_tx_voltage(self.data_buffer)
        msg.beam_dist_mode = self.p.parse_beam_dist_mode(self.data_buffer)
        msg.sonar_mode = self.p.parse_sonar_mode(self.data_buffer)
        msg.gate_tilt = self.p.parse_gate_tilt(self.data_buffer)
        msg.watercolumn_raw = self._build_watercolumn_raw_image()
        msg.watercolumn_beam_directions = self.p.parse_directions(self.data_buffer)
        self.watercolumn_pub.publish(msg)
        # Reset the data buffer.
        self.data_buffer = b''

    def _build_watercolumn_raw_image(self):
        # TODO: dynamically change image dencoding and data scaling
        # based on the data type.
        image = Image()
        image.header.stamp = self.get_clock().now().to_msg()
        image.height = self.p.parse_num_samples(self.data_buffer) # M
        image.width = self.p.parse_num_beams(self.data_buffer) # N
        image.encoding = 'mono8'
        image.step = image.width * 1
        pixel_data = self.p.parse_pixel_data(self.data_buffer)
        pixel_data = np.array(pixel_data)
        image_max = np.max(pixel_data)
        image.data = ((255/image_max) * pixel_data).astype('uint8').tolist()
        return image


def main(args=None):
    """
    Main method for the ROS node.
    """
    rclpy.init(args=args)

    watercolumn_parser = WaterColumnNode()
    rclpy.spin(watercolumn_parser)
    watercolumn_parser.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

