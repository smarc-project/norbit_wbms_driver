#!/usr/bin/env python3



import socket

import rospy
from std_msgs.msg import Float32
from norbit_wbms_driver.cfg import wbms_paramsConfig 
from rospy_message_converter import message_converter
from norbit_wbms_driver.msg import Configs

from copy import deepcopy

def create_configs():
    configlist = []
    config1 = wbms_paramsConfig.defaults.copy()
    
    #Standard stuff
    config1['set_power_mode'] = 1
    config1['set_range_min'] = 0
    config1['set_range_max'] = 200
    config1['set_opening_angle'] = 150
    config1['set_vertical_resolution'] = 2048
    config1['set_horizontal_resolution'] = 512
    config1['set_tx_amp'] = 15
    config1['set_rate'] = 5
    config1['set_tx_Frequency'] = 400

    #stuff to change
    config1['set_mf_decimation'] = 1
    config1['set_tx_Bandwidth'] = 80
    config1['set_tx_pulse_length'] = 500
    config1['set_gain'] = 0
    config1['set_vga'] = 50

    #decimation test
    config02 = deepcopy(config1)
    config02['set_mf_decimation'] = 8
    config03 = deepcopy(config1)
    config03['set_mf_decimation'] = 16

    #tx bandwidth
    config04 = deepcopy(config1)
    config04['set_tx_Bandwidth'] = 40
    config05 = deepcopy(config1)
    config05['set_tx_Bandwidth'] = 20
    config06 = deepcopy(config1)
    config06['set_tx_Bandwidth'] = 0

    #pulse length 10-819
    config07 = deepcopy(config1)
    config07['set_tx_pulse_length'] = 10
    config08 = deepcopy(config1)
    config08['set_tx_pulse_length'] = 100
    config09 = deepcopy(config1)
    config09['set_tx_pulse_length'] = 250
    config10 = deepcopy(config1)
    config10['set_tx_pulse_length'] = 800

    #gain
    config11 = deepcopy(config1)
    config11['set_gain'] = 0
    config11['set_vga'] = 0
    config12 = deepcopy(config1)
    config12['set_gain'] = 33
    config12['set_vga'] = 0
    config13 = deepcopy(config1)
    config13['set_gain'] = 66
    config13['set_vga'] = 0
    config14 = deepcopy(config1)
    config14['set_gain'] = 100
    config14['set_vga'] = 0

    #VGA
    config15 = deepcopy(config1)
    config15['set_vga'] = 25
    config16 = deepcopy(config1)
    config16['set_vga'] = 50
    config17 = deepcopy(config1)
    config17['set_vga'] = 75
    config18 = deepcopy(config1)
    config18['set_vga'] = 100
    
    configlist.append(config1)
    configlist.append(config02)
    configlist.append(config03)
    configlist.append(config04)
    configlist.append(config05)
    configlist.append(config06)
    configlist.append(config07)
    configlist.append(config08)
    configlist.append(config09)
    configlist.append(config10)
    configlist.append(config11)
    configlist.append(config12)
    configlist.append(config13)
    configlist.append(config14)
    configlist.append(config15)
    configlist.append(config16)
    configlist.append(config17)
    configlist.append(config18)

    return configlist




def main():
    """
    Main method for the ROS node.
    """
    rospy.init_node('cycler')
    pub = rospy.Publisher('configs', Configs, queue_size=10)
    dictlist = create_configs()
    iter = 0
    interval = 10
    rate = rospy.Rate(1/interval)
    rate.sleep()
    while not rospy.is_shutdown():
        if iter >len(dictlist)-1:
            iter = 0
        msg = message_converter.convert_dictionary_to_ros_message('norbit_wbms_driver/Configs', dictlist[iter])
        iter = iter + 1
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()
