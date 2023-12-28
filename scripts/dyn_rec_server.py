#!/usr/bin/env python3



import socket

import rospy
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
from norbit_wbms_driver.cfg import wbms_paramsConfig 
from norbit_wbms_driver.msg import Configs
from rospy_message_converter import message_converter


class CommandInterface:
    """
    Class that handles the communication with the Norbit sonar.
    """

    # Sonar's IP and port.
    sonar_IP = "192.168.53.53"
    #sonar_IP = "192.168.1.89"
    # Water column data port.
    sonar_PORT = 2209

    

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
        self.initializing = True
        self.cycling = False
        self.original_config = wbms_paramsConfig.defaults.copy()
        self.config_server = Server(wbms_paramsConfig, self.dynamic_callback)
        self.config_sub = rospy.Subscriber('configs', Configs, self.cycle_callback)
        
       
      


    def dynamic_callback(self, config, level):
        """
        Callback function for the dynamic reconfigure server.
        """
        if not self.initializing and not self.cycling:
            self.find_change_and_send(self.original_config, config, True)
        self.cycling = False
        self.original_config = config.copy()
        return config
    
    def send_whole_config(self,conf): 
        passinglist =["set_range_min", "set_range_max", "set_range_R0","set_range_R1", "set_depth_D0", "set_depth_D1", "set_horizontal_resolution", "set_tx_Bandwidth", "set_tx_amp", "set_tx_pulse_length"] 
        for key in conf:
            if key in passinglist:
                continue
            if key == "set_gate_mode":
                msg = key + " " + str(conf[key])
                self.send_msg(msg)
                if conf[key] == 2:
                    msg = "set_range" + " " + str(conf["set_range_R0"]) + " " + str(conf["set_range_R1"]) + " " +  str(conf["set_depth_D0"]) + " " + str(conf["set_depth_D1"])
                else:
                    msg = "set_range" + " " + str(conf["set_range_min"]) + " " + str(conf["set_range_max"])
                self.send_msg(msg)
            elif key == "set_vertical_resolution":
                msg = "set_resolution" + " " + str(conf[key]) + " " + str(conf["set_horizontal_resolution"])
                self.send_msg(msg)
            elif key == "set_tx_Frequency":
                msg = "set_tx" + " " + str(conf[key]) + " " + str(conf["set_tx_Bandwidth"]) + " " + str(conf["set_tx_amp"]) + " " + str(conf["set_tx_pulse_length"])
                self.send_msg(msg)
            elif key == "set_power_mode":
                if conf[key]==0:
                    msg = "off"
                else:
                    msg = "on"
                self.send_msg(msg)
            else:
                msg = key + " " + str(conf[key])
                self.send_msg(msg)
        self.cycling = True
        self.config_server.update_configuration(conf)
        
    def find_change_and_send(self, config1, config2, with_break):
        msg = ""
        for key in config1:
            if key == "groups":
                break
            if config1[key] != config2[key]:
                if key == 'set_gate_mode':
                    msg = key + " " + str(config2[key])
                    self.send_msg(msg)
                    if config2[key] == 2:
                        msg = "set_range" + " " + str(config2["set_range_R0"]) + " " + str(config2["set_range_R1"]) + " " +  str(config2["set_depth_D0"]) + " " + str(config2["set_depth_D1"])
                        config1["set_range_R0"], config1["set_range_R1"], config1["set_depth_D0"], config1["set_depth_D1"] = config2["set_range_R0"], config2["set_range_R1"], config2["set_depth_D0"], config2["set_depth_D1"]
                    else:
                        msg = "set_range" + " " + str(config2["set_range_min"]) + " " + str(config2["set_range_max"])
                        config1["set_range_min"], config1["set_range_max"] = config2["set_range_min"], config2["set_range_max"]
                    self.send_msg(msg)
        
                elif (key == "set_range_min" or key == "set_range_max"):
                    if config2["set_gate_mode"]!=2:
                        msg = "set_range" + " " + str(config2["set_range_min"]) + " " + str(config2["set_range_max"])
                        config1["set_range_min"], config1["set_range_max"] = config2["set_range_min"], config2["set_range_max"]
                        self.send_msg(msg)
                
                elif (key == "set_range_R0" or key == "set_range_R1" or key == "set_depth_D0" or key == "set_depth_D1"):
                    if config2["set_gate_mode"]==2:
                        msg = "set_range" + " " + str(config2["set_range_R0"]) + " " + str(config2["set_range_R1"]) + " " +  str(config2["set_depth_D0"]) + " " + str(config2["set_depth_D1"])
                        config1["set_range_R0"], config1["set_range_R1"], config1["set_depth_D0"], config1["set_depth_D1"] = config2["set_range_R0"], config2["set_range_R1"], config2["set_depth_D0"], config2["set_depth_D1"]
                        self.send_msg(msg)
                
                elif key == "set_vertical_resolution" or key == "set_horizontal_resolution":
                    msg = "set_resolution" + " " + str(config2["set_vertical_resolution"]) + " " + str(config2["set_horizontal_resolution"])
                    config1["set_vertical_resolution"], config1["set_horizontal_resolution"] = config2["set_vertical_resolution"], config2["set_horizontal_resolution"]
                    self.send_msg(msg)
                
                elif key == "set_tx_Frequency" or key == "set_tx_Bandwidth" or key == "set_tx_amp" or key == "set_tx_pulse_length":
                    msg = "set_tx" + " " + str(config2["set_tx_Frequency"]) + " " + str(config2["set_tx_Bandwidth"]) + " " + str(config2["set_tx_amp"]) + " " + str(config2["set_tx_pulse_length"])
                    config1["set_tx_Frequency"], config1["set_tx_Bandwidth"], config1["set_tx_amp"], config1["set_tx_pulse_length"] = config2["set_tx_Frequency"], config2["set_tx_Bandwidth"], config2["set_tx_amp"], config2["set_tx_pulse_length"]
                    self.send_msg(msg)
                elif key == "set_power_mode":
                    if config2[key]==0:
                        msg = "off"
                    else:
                        msg = "on"
                    config1[key] = config2[key]
                    self.send_msg(msg)
                else:
                    msg = key + " " + str(config2[key])
                    self.send_msg(msg)
                    config1[key] = config2[key]
                if with_break:
                    break



    def cycle_callback(self, msg):
        conf = message_converter.convert_ros_message_to_dictionary(msg)
        self.find_change_and_send(self.original_config, conf, False)
        self.cycling = True
        self.config_server.update_configuration(conf)
    
    
    
    def send_msg(self,msg):
        # rospy.loginfo(msg)
        self.tcp_sock.send(msg.encode())
        reply = self.tcp_sock.recv(1024)
        # rospy.loginfo(reply)

# first make a function (send_whole_config) then a function (send_and_receive)
def main():
    """
    Main method for the ROS node.
    """
    rospy.init_node('wbms_command_interface')
    rospy.loginfo("Starting the FLS parsing node...")
    passinglist =["set_range_min", "set_range_max", "set_range_R0","set_range_R1", "set_depth_D0", "set_depth_D1", "set_horizontal_resolution", "set_tx_Bandwidth", "set_tx_amp", "set_tx_pulse_length"] 
    comms = CommandInterface()
    conf = wbms_paramsConfig.defaults.copy()
    comms.send_whole_config(conf)  
    comms.initializing = False
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    main()
