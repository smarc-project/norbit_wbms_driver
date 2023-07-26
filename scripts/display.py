#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
import tf_conversions
import tf2_ros
import math
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import numpy as np
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import cv2
from cv_bridge import CvBridge, CvBridgeError
from Norbit_FLS_driver.cfg import fls_paramsConfig 
from Norbit_FLS_driver.msg import Fls


def image_callback(msg:Image):
    global num_beams, beam_samples
    
    data = msg.data

    # Separate the data into imagearray and directions
    imagearray = np.array(data[:-num_beams], dtype=np.float32)
    imagearray = imagearray.reshape((beam_samples, num_beams))
    directions = np.array(data[-num_beams:], dtype=np.float32) + math.pi/2

    # Normalize the imagearray between 0 and 255
    max_value = np.max(imagearray)
    imagearray = (255 / max_value) * imagearray

    # Calculate the final indices without explicit loops
    j = np.arange(beam_samples)[:, np.newaxis]
    final_array_x = np.round(beam_samples + j * np.cos(directions)).astype(int)
    final_array_y = np.round(beam_samples - j * np.sin(directions)).astype(int)

    # Clip the indices to stay within the final_array dimensions
    final_array_x = np.clip(final_array_x, 0, beam_samples*2-1)
    final_array_y = np.clip(final_array_y, 0, beam_samples-1)

    # Create the final_array using indexing
    final_array = np.zeros((beam_samples, beam_samples*2), dtype=np.uint8)
    final_array[final_array_y, final_array_x] = imagearray[np.arange(beam_samples)[:, np.newaxis], np.arange(num_beams)]

    # Create the final Image message
    final = Image()
    final.data = final_array.flatten().tolist()
    final.width = beam_samples*2
    final.height = beam_samples
    final.encoding = "mono8"
    final.is_bigendian = 0
    final.step = beam_samples*2
    pub.publish(final)

def data_callback(msg):
    global num_beams, beam_samples
    beam_samples= msg.num_samples
    num_beams = msg.num_beams
    




    

rospy.init_node('displayer')
# sub_goal = rospy.Subscriber('/lolo/sim/fls/image', Image, image_callback)
sub_goal = rospy.Subscriber('/fls/raw', Float32MultiArray, image_callback)
sub_parser = rospy.Subscriber('/fls/data', Fls, data_callback)
pub = rospy.Publisher('/fls/display', Image, queue_size=1)
beam_samples=650
num_beams = 256

if __name__ == '__main__':
    rospy.spin()
