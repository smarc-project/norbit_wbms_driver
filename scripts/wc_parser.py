#!/usr/bin/env python3
"""
norbit data parser.
"""
import struct
import socket
import rospy
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from norbit_wbms_driver.msg import WaterColumn
import rosparam

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



class WbmsNode:
    """
    Class to handle the HEX parsing from an norbit sonar's data stream.
    """

    BUFFER_SIZE_BYTES = 512000

    def __init__(self, ip, port):
        self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        while not rospy.is_shutdown():
            try:
                self.tcp_sock.connect((ip, port))
                rospy.loginfo("TCP socket successfully bound to: %s:%i", ip,
                            port)
                break
            except:
                rospy.logerr("Failed to bind socket to %s:%s. Check ethernet configuration \
                            and restart the node.", ip, port)
                rospy.sleep(1)
                #raise

        # Set a timout for the socket.
        self.tcp_sock.settimeout(SOCKET_TIMEOUT)

        # Build our parser.
        self.p = WCParser()

        # Create the buffer where we'll store the data being streamed in.
        self.data_buffer = b''

        # Setup our raw array publisher and the image publisher.
        self.img_pub = rospy.Publisher("wc/image",
                                       Image, queue_size=1)
        #self.raw_pub = rospy.Publisher("wc/raw",
        #                               Float32MultiArray, queue_size=1)
        self.data_pub = rospy.Publisher("wc/data", WaterColumn, queue_size=1)


    def parse_and_publish(self):
        """
        Parse and publish the data recived from the ICP socket.
        """
        # Get data from udp socket
        try:
            # Fetch the initial chunk.
            data, addr = self.tcp_sock.recvfrom(self.BUFFER_SIZE_BYTES)
            # Quick check of the deadbeef.
            if self.p.parse_preamble(data) != 0xDEADBEEF:
                rospy.logerr("Message did not pass the deadbeef check!")
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
                    data, addr = self.tcp_sock.recvfrom(self.BUFFER_SIZE_BYTES)
                    self.data_buffer += data
                except:
                    print("something went wrong in the msg reconstruction loop")

        except socket.timeout:
            rospy.logerr("Watercolumn interface socket timed out, verify connection.")
            return

        print("Final size of the concat msg={}".format(len(self.data_buffer)))

        # Get the pixel and beam directions arrays.
        pixel_data = self.p.parse_pixel_data(self.data_buffer)
        directions = self.p.parse_directions(self.data_buffer)

        # Build the image.
        image = Image()
        image.header.stamp = rospy.Time.now()
        image.height = self.p.parse_num_samples(self.data_buffer)  # M
        image.width = self.p.parse_num_beams(self.data_buffer)     # N
        image.encoding = 'mono8'  # TODO This should dynamically change
                                   # depending on the data's dtype...
        image.step = image.width * 1
        # image.data =  np.array(pixel_data, dtype='uint8').tolist()
        image.data = np.array(pixel_data)
        image_max = np.max(image.data)
        image.data = ((255/image_max) * image.data).astype('uint8').tolist()
        self.img_pub.publish(image)

        # Build the multi array.
        array = Float32MultiArray()
        img_dim = MultiArrayDimension()
        img_dim.label = "pixel_data"
        img_dim.size = image.height * image.width
        img_dim.stride = img_dim.size * 4
        dir_dim = MultiArrayDimension()
        dir_dim.label = "directions"
        dir_dim.size = len(directions)
        dir_dim.stride = dir_dim.size * 4
        array.layout.dim = [img_dim, dir_dim]
        array.data = np.array(pixel_data).astype(float).tolist() + directions
        # self.raw_pub.publish(array)

        # build the WaterColumn msg
        msg = WaterColumn()
        msg.preamble = self.p.parse_preamble(self.data_buffer)
        msg.type = self.p.parse_type(self.data_buffer)
        msg.size = self.p.parse_size(self.data_buffer)
        msg.snd_velocity = self.p.parse_sound_velocity(self.data_buffer)        
        msg.sample_rate = self.p.parse_sample_rate(self.data_buffer)
        msg.num_beams = self.p.parse_num_beams(self.data_buffer)
        msg.num_samples = self.p.parse_num_samples(self.data_buffer)
        msg.Time = self.p.parse_timestamp(self.data_buffer)
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
        msg.watercolumn_raw = array
        self.data_pub.publish(msg)
        # Reset the data buffer.
        self.data_buffer = b''





def main():
    """
    Main method for the ROS node.
    """
    rospy.init_node('wc_parser')
    rospy.loginfo("Starting the WBMS water column parsing node...")
    
    # sonar's IP and port.
    sonar_IP = rosparam.get_param(rospy.get_name() + '/wbms_sonar_ip')
    # Water column data port.
    sonar_PORT = rosparam.get_param(rospy.get_name() + '/wbms_watercolumn_data_port')
    
    wbms = WbmsNode(sonar_IP, sonar_PORT)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        wbms.parse_and_publish()
        rate.sleep()

if __name__ == "__main__":
    main()

