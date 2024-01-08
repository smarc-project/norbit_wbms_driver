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
from norbit_wbms_driver.msg import Bathymetry
from norbit_wbms_driver.msg import Bathymetry_beam
from sensor_msgs.msg import LaserScan

__author__ = "Aldo Teran"
__author_email = "aldot@kth.se"
__license__ = "MIT"
__status__ = "Development"



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
            bathy_msg.beams.append(Bathymetry_beam(
                sample_num,
                angle,
                upper_gate,
                lower_gate,
                intensity,
                flags,
                quality_flag,
                quality_value,
                range_
            ))
            


class BathymetryNode:
    """
    Class to handle the HEX parsing from an norbit sonar's data stream.
    """

    # sonar's IP and port.
    #sonar_IP = "192.168.1.89"
    sonar_IP = "192.168.53.53"
    # Water column data port.
    sonar_PORT = 2210

    BUFFER_SIZE_BYTES = 512000

    def __init__(self):
        self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        while not rospy.is_shutdown():
            try:
                self.tcp_sock.connect((self.sonar_IP, self.sonar_PORT))
                rospy.loginfo("TCP socket successfully bound to: %s:%i", self.sonar_IP,
                            self.sonar_PORT)
                break
            except:
                rospy.logerr("Failed to bind socket to %s:%s. Check ethernet configuration \
                            and restart the node.", self.sonar_IP, self.sonar_PORT)
                rospy.sleep(1)
                #raise

        # Set a timout for the socket.
        self.tcp_sock.settimeout(2)

        # Build our parser.
        self.p = BathymetryParser()

        # Create the buffer where we'll store the data being streamed in.
        self.data_buffer = b''

        # Setup publisher.
        self.bathymetry_pub = rospy.Publisher("wbms/bathymetry",
                                       Bathymetry, 
                                       queue_size=1)
        
        self.laser_scan_pub = rospy.Publisher("wbms/laser_scan_bathymetry", 
                                              LaserScan, 
                                              queue_size=1)


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
            N = self.p.parse_num_beams(data)

            expected_size_bytes = 111 + N*20 # TODO: Double check 111 or 112
            
            real_size = self.p.parse_size(data)

            # Keep looping until the full message is completely received.
            while len(self.data_buffer) < expected_size_bytes:
                try:
                    data, addr = self.tcp_sock.recvfrom(self.BUFFER_SIZE_BYTES)
                    self.data_buffer += data
                except:
                    print("something went wrong in the msg reconstruction loop")

        except socket.timeout:
            rospy.logerr("Command interface socket timed out, verify connection.")
            return

        print("Final size of the concat msg={}".format(len(self.data_buffer)))

        # Build the Bathymetry message
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

        self.bathymetry_pub.publish(msg)
        # Reset the data buffer.
        self.data_buffer = b''


def main():
    """
    Main method for the ROS node.
    """
    rospy.init_node('wbms_parser')
    rospy.loginfo("Starting the WBMS sonar parsing node...")
    wbms = BathymetryNode()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        wbms.parse_and_publish()
        rate.sleep()

if __name__ == "__main__":
    main()

