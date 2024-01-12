#!/usr/bin/env python3
import rospy
import norbit_wbms_driver.srv
from std_msgs.msg import String 
import rosparam
import sys

def main():
    print("Test client started")
    rospy.init_node('wbms_configure_testclient')
    configService = rospy.ServiceProxy('/wbms_configure', norbit_wbms_driver.srv.WbmsConfigure)

    print("Waiting for service")
    rospy.wait_for_service('/wbms_configure', timeout=None)

    print("Calling service with config: " + str(sys.argv[1]))
    rsp = configService(int(sys.argv[1]))
    print("Service response: " + rsp.ConfigResponse)

    #rospy.spin()

if __name__ == "__main__":
    main()