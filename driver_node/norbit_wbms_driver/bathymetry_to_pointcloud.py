import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from norbit_wbms_interfaces.msg import Bathymetry, BathymetryBeam
from sensor_msgs.msg import PointCloud2

class Translator_node(Node):

    def __init__(self):
        super().__init__('bathymetry_to_pointcloud')

        input_topic = "input"
        output_topic = "output"

        self.declare_parameter("input_topic", "bathymetry")
        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value

        self.declare_parameter("output_topic", "bathymetry/points")
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value


        self.declare_parameter("frame_id", "mbes_link")
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        #Elevon port fb
        self.pub_pc2 = self.create_publisher(PointCloud2, output_topic, 2)
        self.sub_bathymetry = self.create_subscription(Bathymetry,input_topic,self.callback_bathmetry,1)


    def callback_bathmetry(self, msg):
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
        pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
        pointcloud_msg.header.frame_id = self.frame_id

        self.pub_pc2.publish(pointcloud_msg)




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
