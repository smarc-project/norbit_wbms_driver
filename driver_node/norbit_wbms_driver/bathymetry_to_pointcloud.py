import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from norbit_wbms_interfaces.msg import Bathymetry, BathymetryBeam
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

class Translator_node(Node):

    def __init__(self):
        super().__init__('bathymetry_to_pointcloud')

        input_topic = "input"
        output_topic = "output"

        self.declare_parameter("input_topic", "/lolo/sensors/mbes/bathymetry")
        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value

        self.declare_parameter("output_topic", "bathymetry/points")
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value


        self.declare_parameter("frame_id", "lolo/base_link")
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        #Elevon port fb
        self.pub_pc2 = self.create_publisher(PointCloud2, output_topic, 2)
        self.sub_bathymetry = self.create_subscription(Bathymetry,input_topic,self.callback_bathmetry,1)


    def callback_bathmetry(self, in_msg):
        print("Converting Bathymetry msg to pointcloud2 msg.")

        beams = in_msg.beams
        beams.sort(key=lambda x: x.angle)

        angles = [-1*beam.angle for beam in in_msg.beams]
        ranges = [beam.range for beam in in_msg.beams]
        intensities = [beam.intensity for beam in in_msg.beams]

        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
              PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
              PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
              PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)]

        
        z = ranges * np.cos(angles)
        z = z*-1
        y = ranges * np.sin(angles)
        x = np.zeros(len(ranges))
        intensities = np.array(intensities)

        
        points = np.array([x, y, z, intensities]).reshape(4, -1).T
        pc2_msg = point_cloud2.create_cloud(header, fields, points)

        self.pub_pc2.publish(pc2_msg)




def main(args=None):
    rclpy.init(args=args)
    translator = Translator_node()

    executor = MultiThreadedExecutor()
    executor.add_node(translator)
    executor.spin()
    #rclpy.spin(translator)

    translator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
