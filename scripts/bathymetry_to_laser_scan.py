#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from norbit_wbms_driver.msg import Bathymetry
from norbit_wbms_driver.msg import Bathymetry_beam


def bathymetry_to_laser_scan(in_msg: Bathymetry):
    print("Converting Bathymetry msg to LaserScan msg.")
    out_msg = LaserScan()

    out_msg.angle_min = in_msg.swath_dir - in_msg.swath_open/2
    out_msg.angle_max = in_msg.swath_dir + in_msg.swath_open/2
    
    out_msg.header.stamp = rospy.Time.now()
    out_msg.header.frame_id = "lolo/mbes_link"
    
    out_msg.angle_increment = in_msg.swath_open / in_msg.num_beams
    out_msg.time_increment = 0
    out_msg.scan_time = 1 / in_msg.ping_rate

    beams = in_msg.beams
    beams.sort(key=lambda x: x.angle)

    #angles = [beam.angle for beam in in_msg.beams]
    #for i in range(len(angles)-1):
    #    if angles[i] > angles[i+1]:
    #        print("Warning: Angles in bathymetry msg not strictly increasing")
    
    ranges = [beam.range for beam in in_msg.beams]
    intensities = [beam.intensity for beam in in_msg.beams]

    out_msg.range_max = max(ranges)
    out_msg.range_min = min(ranges)
    out_msg.ranges = ranges
    out_msg.intensities = intensities

    laser_scan_pub.publish(out_msg)


if __name__ == "__main__":
    rospy.init_node("bathymetry_to_laser_scan")

    bathymetry_sub = rospy.Subscriber("bathymetry", 
                                      Bathymetry, 
                                      bathymetry_to_laser_scan, 
                                      queue_size=1)      
        
    laser_scan_pub = rospy.Publisher("laser_scan_bathymetry", LaserScan, queue_size=1)

    while not rospy.is_shutdown():
        rospy.spin()