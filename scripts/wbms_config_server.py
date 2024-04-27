#!/usr/bin/env python3
import rospy
import norbit_wbms_driver.srv
from std_msgs.msg import String 
import subprocess
import os
import rosparam

import wbms_configure

cfg_dir = ""
default_cfg = ""
short_range_cfg = ""
medium_range_cfg = ""
long_range_cfg = ""

# TODO: Make a class and move the code from wbms_configure.py into it

def send_configuration(req: norbit_wbms_driver.srv.WbmsConfigureRequest):
    cfg_dir = rosparam.get_param(rospy.get_name() + '/cfg_dir')
    cfg1 = rosparam.get_param(rospy.get_name() + '/cfg1')
    cfg2 = rosparam.get_param(rospy.get_name() + '/cfg2')
    cfg3 = rosparam.get_param(rospy.get_name() + '/cfg3')
    cfg4 = rosparam.get_param(rospy.get_name() + '/cfg4')
    ip = rosparam.get_param(rospy.get_name() + '/ip')
    port = rosparam.get_param(rospy.get_name() + '/port')

    rospy.loginfo("Setting WBMS cfg: " + str(req.ConfigRequest))

    rsp = 99

    if req.ConfigRequest == 0:
        print("Turning off sonar.")
        rsp = wbms_configure.send_configuration("", ip, port)
    if req.ConfigRequest == 1:
        print("Setting conf file: " + cfg_dir + cfg1)
        rsp = wbms_configure.send_configuration(cfg_dir + cfg1, ip, port)
    elif req.ConfigRequest == 2:
        print("Setting conf file: " + cfg_dir + cfg2)
        rsp = wbms_configure.send_configuration(cfg_dir + cfg2, ip, port)
    elif req.ConfigRequest == 3:
        print("Setting conf file: " + cfg_dir + cfg3)
        rsp = wbms_configure.send_configuration(cfg_dir + cfg3, ip, port)
    elif req.ConfigRequest == 4:
        print("Setting conf file: " + cfg_dir + cfg4)
        rsp = wbms_configure.send_configuration(cfg_dir + cfg4, ip, port)

    return norbit_wbms_driver.srv.WbmsConfigureResponse(str(rsp))

def main():
    print("Configure service started")
    # Params: sonar ip, sonar port
    print(rospy.get_param_names())
    
    rospy.init_node('wbms_configure_server')

    topic = rosparam.get_param(rospy.get_name() + '/service_topic')
    configServ = rospy.Service(topic, norbit_wbms_driver.srv.WbmsConfigure, send_configuration)

    print("Turning off sonar.")
    ip = rosparam.get_param(rospy.get_name() + '/ip')
    port = rosparam.get_param(rospy.get_name() + '/port')
    rsp = wbms_configure.send_configuration("", ip, port)
    rospy.spin()

if __name__ == "__main__":
    main()