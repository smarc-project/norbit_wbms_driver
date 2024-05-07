#! /usr/bin/env python3

import rospy
from norbit_wbms_driver.msg import Bathymetry
from norbit_wbms_driver.msg import Bathymetry_beam
from sensor_msgs.msg import PointCloud2
import numpy as np
import ros_numpy
import tf2_ros
import tf


class wbmsToPtcloud:
    """
    Class to turn the MBES laserscan message to a poitncloud2.
    """
    def __init__(self):
        

        self.pointcloud_pub = rospy.Publisher("pc2_bathymetry", PointCloud2, queue_size=1)

        rospy.Subscriber("bathymetry", 
                                      Bathymetry, 
                                      self.bathymetry_to_pc2, 
                                      queue_size=1)      

    def bathymetry_to_pc2(self, in_msg: Bathymetry):
        print("Converting Bathymetry msg to pointcloud2 msg.")

        beams = in_msg.beams
        beams.sort(key=lambda x: x.angle)

        #angles = [beam.angle for beam in in_msg.beams]
        #for i in range(len(angles)-1):
        #    if angles[i] > angles[i+1]:
        #        print("Warning: Angles in bathymetry msg not strictly increasing")
        
        angles = [beam.angle for beam in in_msg.beams]
        ranges = [beam.range for beam in in_msg.beams]
        intensities = [beam.intensity for beam in in_msg.beams]

        angles = np.asarray(angles)
        ranges = np.asarray(ranges)
        intensities = np.asarray(intensities)

        # Build the recarray with points and intensities
        pointcloud = np.recarray((1,len(ranges)),dtype=[('x', np.float32),
                                                        ('y', np.float32),
                                                        ('z', np.float32),
                                                        ('intensity', np.uint8)])
        pointcloud['z'] = np.zeros(len(ranges))
        pointcloud['y'] = ranges * np.sin(angles)
        pointcloud['x'] = ranges * np.cos(angles)
        pointcloud['intensity'] = intensities

        # Do some magic
        pointcloud_msg = ros_numpy.point_cloud2.array_to_pointcloud2(pointcloud)
        pointcloud_msg.header.stamp = rospy.Time.now()
        pointcloud_msg.header.frame_id = 'lolo/mbes_link'

        self.pointcloud_pub.publish(pointcloud_msg)


if __name__ == "__main__":
    rospy.init_node("pc2_to_laser_scan")

    
    converter = wbmsToPtcloud()
    

    while not rospy.is_shutdown():
        rospy.spin()