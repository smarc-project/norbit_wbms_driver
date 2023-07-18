#!/usr/bin/env python3



import socket

import rospy
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
from dyn_rec.cfg import fls_paramsConfig 



class CommandInterface:
    """
    Class that handles the communication with the FLS.
    """

    # FLS's IP and port.
    FLS_IP = "192.168.1.89"
    # Water column data port.
    FLS_PORT = 2209

    

    def __init__(self):
        self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        while not rospy.is_shutdown():
            try:
                self.tcp_sock.connect((self.FLS_IP, self.FLS_PORT))
                rospy.loginfo("TCP socket successfully bound to: %s:%i", self.FLS_IP,
                            self.FLS_PORT)
                break
            except:
                rospy.logerr("Failed to bind socket to %s:%s. Check ethernet configuration \
                            and restart the node.", self.FLS_IP, self.FLS_PORT)
                rospy.sleep(1)
                #raise

        # Set a timout for the socket.
        self.tcp_sock.settimeout(2)

     
        self.config_server = Server(fls_paramsConfig, self.dynamic_callback)

       
      


    def dynamic_callback(self, config, level):
        """
        Callback function for the dynamic reconfigure server.
        """
        passinglist =["set_range_min", "set_range_max", "set_range_R0","set_range_R1", "set_depth_D0", "set_depth_D1", "set_horizontal_resolution", "set_tx_Bandwidth", "set_tx_amp", "set_tx_pulse_length"]                                                      
        gate_nr = config["set_gate_mode"]



        for key in config:
            if key in passinglist:
                continue
            if key == "set_gate_mode":
                msg = key + str(config[key])
                self.tcp_sock.send(msg.encode())
                if gate_nr == 2:
                    msg = "set_range" + " " + str(config["set_range_R0"]) + " " + str(config["set_range_R1"]) + " " +  str(config["set_depth_D0"]) + " " + str(config["set_depth_D1"])
                else:
                    msg = "set_range" + " " + str(config["set_range_min"]) + " " + str(config["set_range_max"])
                self.tcp_sock.send(msg.encode())
            elif key == "set_vertical_resolution":
                msg = "set_resolution" + " " + str(config[key]) + " " + str(config["set_horizontal_resolution"])
            elif key == "set_tx_Frequency":
                msg = "set_tx" + " " + str(config[key]) + " " + str(config["set_tx_Bandwidth"]) + " " + str(config["set_tx_amp"]) + " " + str(config["set_tx_pulse_length"])
                self.tcp_sock.send(msg.encode())
            else:
                msg = key + " " + str(config[key])
                self.tcp_sock.send(msg.encode())
        return config





def main():
    """
    Main method for the ROS node.
    """
    rospy.init_node('fls_command_interface')
    rospy.loginfo("Starting the FLS parsing node...")
    fls_comms = CommandInterface()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    main()
