#!/usr/bin/env python3



import socket

import rospy
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
from Norbit_FLS_driver.cfg import fls_paramsConfig 



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
        self.initializing = True
        self.original_config = fls_paramsConfig.defaults
        rospy.loginfo(self.original_config)
        self.config_server = Server(fls_paramsConfig, self.dynamic_callback)
        
       
      


    def dynamic_callback(self, config, level):
        """
        Callback function for the dynamic reconfigure server.
        """
        if not self.initializing:
            msg=""
            changed_gate_mode = False
            for key in config:
                if key in self.original_config:
                    if config[key] != self.original_config[key]:
                        if key == "set_gate_mode":
                            changed_gate_mode = True
                            if config[key] == 2:
                                msg = "set_range" + " " + str(config["set_range_R0"]) + " " + str(config["set_range_R1"]) + " " +  str(config["set_depth_D0"]) + " " + str(config["set_depth_D1"])
                            else:
                                msg = "set_range" + " " + str(config["set_range_min"]) + " " + str(config["set_range_max"])
                        elif (key == "set_range_min" or key == "set_range_max"):
                            if config["set_gate_mode"]!=2:
                                msg = "set_range" + " " + str(config["set_range_min"]) + " " + str(config["set_range_max"])
                        elif (key == "set_range_R0" or key == "set_range_R1" or key == "set_depth_D0" or key == "set_depth_D1"):
                            if config["set_gate_mode"]==2:
                                msg = "set_range" + " " + str(config["set_range_R0"]) + " " + str(config["set_range_R1"]) + " " +  str(config["set_depth_D0"]) + " " + str(config["set_depth_D1"])
                        elif key == "set_vertical_resolution" or key == "set_horizontal_resolution":
                            msg = "set_resolution" + " " + str(config["set_vertical_resolution"]) + " " + str(config["set_horizontal_resolution"])
                        elif key == "set_tx_Frequency" or key == "set_tx_Bandwidth" or key == "set_tx_amp" or key == "set_tx_pulse_length":
                            msg = "set_tx" + " " + str(config["set_tx_Frequency"]) + " " + str(config["set_tx_Bandwidth"]) + " " + str(config["set_tx_amp"]) + " " + str(config["set_tx_pulse_length"])
                        else:
                            msg = key + " " + str(config[key])
                        # rospy.loginfo(key)
                        self.original_config = config
                        break
            if changed_gate_mode:
                msg_gate = "set_gate_mode" + " " + str(config["set_gate_mode"])
                self.tcp_sock.send(msg_gate.encode())
                reply_gate = self.tcp_sock.recv(1024)
                #rospy.loginfo(msg_gate)
                #rospy.loginfo(reply_gate)
            #rospy.loginfo(msg)
            self.tcp_sock.send(msg.encode())
            reply = self.tcp_sock.recv(1024)
            #rospy.loginfo(reply)
        return config
        
        # passinglist =["set_range_min", "set_range_max", "set_range_R0","set_range_R1", "set_depth_D0", "set_depth_D1", "set_horizontal_resolution", "set_tx_Bandwidth", "set_tx_amp", "set_tx_pulse_length"]                                                      
        # gate_nr = config["set_gate_mode"]



        # for key in config:
        #     if key in passinglist:
        #         continue
        #     if key == "set_gate_mode":
        #         msg = key + str(config[key])
        #         #self.tcp_sock.send(msg.encode())
        #         print(msg)
        #         if gate_nr == 2:
        #             msg = "set_range" + " " + str(config["set_range_R0"]) + " " + str(config["set_range_R1"]) + " " +  str(config["set_depth_D0"]) + " " + str(config["set_depth_D1"])
        #         else:
        #             msg = "set_range" + " " + str(config["set_range_min"]) + " " + str(config["set_range_max"])
        #         #self.tcp_sock.send(msg.encode())
        #         print(msg)
        #     elif key == "set_vertical_resolution":
        #         msg = "set_resolution" + " " + str(config[key]) + " " + str(config["set_horizontal_resolution"])
        #     elif key == "set_tx_Frequency":
        #         msg = "set_tx" + " " + str(config[key]) + " " + str(config["set_tx_Bandwidth"]) + " " + str(config["set_tx_amp"]) + " " + str(config["set_tx_pulse_length"])
        #         #self.tcp_sock.send(msg.encode())
        #         print(msg)
        #     else:
        #         msg = key + " " + str(config[key])
        #         #self.tcp_sock.send(msg.encode())
        #         print(msg)
        return config





def main():
    """
    Main method for the ROS node.
    """
    rospy.init_node('fls_command_interface')
    rospy.loginfo("Starting the FLS parsing node...")
    passinglist =["set_range_min", "set_range_max", "set_range_R0","set_range_R1", "set_depth_D0", "set_depth_D1", "set_horizontal_resolution", "set_tx_Bandwidth", "set_tx_amp", "set_tx_pulse_length"] 
    fls_comms = CommandInterface()
    for key in fls_paramsConfig.defaults:
        if key in passinglist:
            continue
        if key == "set_gate_mode":
            msg = key + " " + str(fls_paramsConfig.defaults[key])
            # rospy.loginfo(msg)
            fls_comms.tcp_sock.send(msg.encode())
            reply = fls_comms.tcp_sock.recv(1024)
            if fls_paramsConfig.defaults[key] == 2:
                msg = "set_range" + " " + str(fls_paramsConfig.defaults["set_range_R0"]) + " " + str(fls_paramsConfig.defaults["set_range_R1"]) + " " +  str(fls_paramsConfig.defaults["set_depth_D0"]) + " " + str(fls_paramsConfig.defaults["set_depth_D1"])
            else:
                msg = "set_range" + " " + str(fls_paramsConfig.defaults["set_range_min"]) + " " + str(fls_paramsConfig.defaults["set_range_max"])
            # rospy.loginfo(msg)
            fls_comms.tcp_sock.send(msg.encode())
            reply = fls_comms.tcp_sock.recv(1024)
        elif key == "set_vertical_resolution":
            msg = "set_resolution" + " " + str(fls_paramsConfig.defaults[key]) + " " + str(fls_paramsConfig.defaults["set_horizontal_resolution"])
            # rospy.loginfo(msg)
            fls_comms.tcp_sock.send(msg.encode())
            reply = fls_comms.tcp_sock.recv(1024)
        elif key == "set_tx_Frequency":
            msg = "set_tx" + " " + str(fls_paramsConfig.defaults[key]) + " " + str(fls_paramsConfig.defaults["set_tx_Bandwidth"]) + " " + str(fls_paramsConfig.defaults["set_tx_amp"]) + " " + str(fls_paramsConfig.defaults["set_tx_pulse_length"])
            rospy.loginfo(msg)
            # fls_comms.tcp_sock.send(msg.encode())
            # reply = fls_comms.tcp_sock.recv(1024)
        else:
            msg = key + " " + str(fls_paramsConfig.defaults[key])
            # rospy.loginfo(msg)
            fls_comms.tcp_sock.send(msg.encode())
            reply = fls_comms.tcp_sock.recv(1024)
    fls_comms.initializing = False    

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    main()
