#!/usr/bin/env python3
"""
norbit data parser.
"""
import rclpy
from rclpy.node import Node
import struct
import socket
import math
import time

from norbit_wbms_interfaces.msg import Bathymetry, BathymetryBeam

SOCKET_TIMEOUT = 5

class BathymetryParser:
    """
    Class for handling the raw logic of parsing a raw data packet.

    Based on Sec. 8.1 of the norbit's TN-180196 document.
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

    ### 
    def parse_ping_number(self, msg):
        '''Returns the ping sequence number.'''
        return struct.unpack('<I', msg[36:40])[0]

    ###
    def parse_time(self, msg):
        '''Returns the ping's Unix timestamp at TX.'''
        return struct.unpack('<d', msg[40:48])[0]

    ###
    def parse_time_net(self, msg):
        '''Returns Unix timestamp as fract (send on network time)'''
        return struct.unpack('<d', msg[48:56])[0]
    
    ###
    def parse_ping_rate(self, msg):
        '''Ping rate in Hz'''
        return struct.unpack('<f', msg[56:60])[0]
    
    ###
    def parse_bathy_type(self,msg):
        '''Bathymetric data type (4)'''
        return struct.unpack('<H', msg[60:62])[0]

    ###
    def parse_gain(self, msg):
        '''Returns the intensity value gain.'''
        return struct.unpack('<f', msg[76:80])[0]
    
    ###
    def parse_swath_dir(self, msg):
        '''Returns the swath center direction in radians.'''
        return struct.unpack('<f', msg[100:104])[0]
    
    ###
    def parse_swath_open(self, msg):
        '''Returns the swath opening angle in radians.'''
        return struct.unpack('<f', msg[104:108])[0]
    
    ###
    def parse_tx_angle(self, msg):
        '''Return the tx elevation steering in radians.'''
        return struct.unpack('<f', msg[72:76])[0]

    ###
    def parse_tx_freq(self, msg):
        '''Return the tx frequency in Hz.'''
        return struct.unpack('<f', msg[80:84])[0]

    ###
    def parse_tx_bw(self, msg):
        '''Returns the transmission bandwidth in kHz.'''
        return struct.unpack('<f', msg[84:88])[0]
    
    ###
    def parse_tx_len(self, msg):
        '''Returns the transmission pulse length in sec.'''
        return struct.unpack('<f', msg[88:92])[0]
    
    ###
    def parse_tx_voltage(self, msg):
        '''Returns the peak voltage signal over ceramics, NaN for non-STX sonars.'''
        return struct.unpack('<f', msg[96:100])[0]
    
    ###
    def parse_beam_dist_mode(self, msg):
        '''Returns the beam distance mode.
        
        The modes are:
            1: 512EA
            2: 256EA
        '''
        return struct.unpack('<B', msg[62:63])[0]
    
    ###
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
        return struct.unpack('<B', msg[63:64])[0]
    
    ###
    def parse_gate_tilt(self, msg):
        '''Returns the gate tilt in radians, for depth gates.'''
        return struct.unpack('<f', msg[108:112])[0]
    
    ### 
    def parse_beams(self, msg, bathy_msg):
        '''Returns beam data.'''
        N = self.parse_num_beams(msg)
        sv = self.parse_sound_velocity(msg)
        sr = self.parse_sample_rate(msg)

        if math.isnan(sv): sv = 1500

        for n in range(1, N):
            sample_num = struct.unpack('<I', msg[92+n*20:96+n*20])[0] 
            angle = struct.unpack('<f', msg[96+n*20:100+n*20])[0] 
            upper_gate = struct.unpack('<H', msg[100+n*20:102+n*20])[0]
            lower_gate = struct.unpack('<H', msg[102+n*20:104+n*20])[0]
            intensity = struct.unpack('<f', msg[104+n*20:108+n*20])[0]
            flags = struct.unpack('<H', msg[108+n*20:110+n*20])[0]
            quality_flag = struct.unpack('<B', msg[110+n*20:111+n*20])[0]
            quality_value = struct.unpack('<B', msg[111+n*20:112+n*20])[0]
            range_ = sample_num*sv/(2*sr)

            beam = BathymetryBeam()
            beam.sample_number = sample_num
            beam.angle = angle
            beam.upper_gate = upper_gate
            beam.lower_gate = lower_gate
            beam.intensity = intensity
            beam.flags = flags
            beam.quality_flag = quality_flag
            beam.quality_val = quality_value
            beam.range = range_
            bathy_msg.beams.append(beam)


class BathymetryNode(Node):
    """
    Class to handle the HEX parsing from an norbit sonar's data stream.
    Note that this node blocks until the TCP connection is established.
    #TODO: test actual parsing with real data
    """
    BUFFER_SIZE_BYTES = 512000

    def __init__(self):
        super().__init__('bathymetry_parser')
        self.get_logger().info("Starting Bathymetry Node")

        self.declare_parameter('sonar_ip', '127.0.0.1')
        self.sonar_ip = self.get_parameter('sonar_ip').get_parameter_value().string_value
        self.declare_parameter('bathy_port', 2210)
        self.bathy_port = self.get_parameter('bathy_port').get_parameter_value().integer_value
        self.declare_parameter('output_topic', 'bathymetry')
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.tcp_retry_every = 5 # seconds
        self.tcp_socket = self.connect_to_sonar()

        # Build our parser.
        self.p = BathymetryParser()

        # Create the buffer where we'll store the data being streamed in.
        self.data_buffer = b''

        # Setup publisher.
        self.bathymetry_pub = self.create_publisher(Bathymetry, self.output_topic, 1)
        self.bathymetry_pub_timer = self.create_timer(0.01, self.parse_and_publish)


    def connect_to_sonar(self):
        """
        Connect to the sonar's IP and port and return the TCP socket.
        Retry until connection is established.
        """
        connected = False

        tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_socket.settimeout(SOCKET_TIMEOUT)
        tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        while not connected and rclpy.ok():
            try:
                tcp_socket.connect((self.sonar_ip, self.bathy_port))
                self.get_logger().info(f"Connected to sonar at {self.sonar_ip}:{self.bathy_port}")
                connected = True
                return tcp_socket
            except Exception as e:
                self.get_logger().error(f"Error connecting to sonar at {self.sonar_ip}:{self.bathy_port}")
                self.get_logger().error(str(e))
                self.get_logger().info(f"Trying again in {self.tcp_retry_every} seconds...")
                time.sleep(self.tcp_retry_every)


    def parse_and_publish(self):
        """
        Parse and publish the data recived from the TCP socket.
        """
        try:
            # Fetch the initial chunk.
            data, addr = self.tcp_socket.recvfrom(self.BUFFER_SIZE_BYTES)
            self.data_buffer += data

            # Quick check of the deadbeef.
            if self.p.parse_preamble(self.data_buffer) != 0xDEADBEEF:
                self.get_logger().error("Message did not pass the deadbeef check!")
                return

            # Get the expected size of the packet.
            N = self.p.parse_num_beams(data)

            expected_size_bytes = 112 + N*20

            real_size = self.p.parse_size(data)

            # Keep looping until the full message is completely received.
            while len(self.data_buffer) < expected_size_bytes and rclpy.ok():
                try:
                    data, addr = self.tcp_socket.recvfrom(self.BUFFER_SIZE_BYTES)
                    self.data_buffer += data
                except:
                    self.get_logger().error("something went wrong in the msg reconstruction loop")

        except socket.timeout:
            self.get_logger().error("Bathymetry interface socket timed out, verify connection.")
            return

        self.get_logger().info("Final size of the concat msg={}".format(len(self.data_buffer)))

        # Build the Bathymetry message
        msg = self._build_bathymetry_msg()
        self.bathymetry_pub.publish(msg)
        # Reset the data buffer.
        self.data_buffer = self.data_buffer[expected_size_bytes:]

    def _build_bathymetry_msg(self):
        msg = Bathymetry()
        msg.preamble = self.p.parse_preamble(self.data_buffer)
        msg.type = self.p.parse_type(self.data_buffer)
        msg.size = self.p.parse_size(self.data_buffer)
        msg.snd_velocity = self.p.parse_sound_velocity(self.data_buffer)        
        msg.sample_rate = self.p.parse_sample_rate(self.data_buffer)
        msg.num_beams = self.p.parse_num_beams(self.data_buffer)

        msg.time = self.p.parse_time(self.data_buffer)
        msg.time_net = self.p.parse_time_net(self.data_buffer)
        msg.ping_rate = self.p.parse_ping_rate(self.data_buffer)
        msg.bathy_type = self.p.parse_bathy_type(self.data_buffer)
        msg.beam_dist_mode = self.p.parse_beam_dist_mode(self.data_buffer)
        msg.sonar_mode = self.p.parse_sonar_mode(self.data_buffer)

        msg.tx_angle = self.p.parse_tx_angle(self.data_buffer)
        msg.gain = self.p.parse_gain(self.data_buffer)
        msg.tx_freq = self.p.parse_tx_freq(self.data_buffer)
        msg.tx_bw = self.p.parse_tx_bw(self.data_buffer)
        msg.tx_len = self.p.parse_tx_len(self.data_buffer)
        msg.tx_voltage = self.p.parse_tx_voltage(self.data_buffer)
        msg.swath_dir = self.p.parse_swath_dir(self.data_buffer)
        msg.swath_open = self.p.parse_swath_open(self.data_buffer)
        msg.gate_tilt = self.p.parse_gate_tilt(self.data_buffer)

        self.p.parse_beams(self.data_buffer, msg)
        return msg

    def destroy_node(self):
        """Detroy the node and close the TCP socket."""
        self.get_logger().info("Shutting down bathymetry parser node...")
        if self.tcp_socket:
            try:
                self.tcp_socket.close()
                self.get_logger().info("TCP socket closed successfully.")
            except Exception as e:
                self.get_logger().error(f"Error closing TCP socket: {str(e)}")
        else:
            self.get_logger().info("No TCP socket to close.")
        super().destroy_node()


def main(args=None):
    """
    Main method for the ROS node.
    """
    rclpy.init(args=args)

    bathy_parser = BathymetryNode()
    try:
        rclpy.spin(bathy_parser)
    except KeyboardInterrupt:
        bathy_parser.get_logger().info("Keyboard interrupt received, shutting down...")
    finally:
        bathy_parser.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

