import rclpy
from rclpy.node import Node
import numpy as np
import cv2

from sensor_msgs.msg import Image
from norbit_wbms_interfaces.msg import WaterColumn


class WaterColumnVisualizer(Node):
    def __init__(self):
        super().__init__("watercolumn_visualizer")
        self.get_logger().info("WaterColumnVisualizer node started")
        self.message_counter = 0
        self.subscription = self.create_subscription(
            WaterColumn, "watercolumn", self.listener_callback, 10
        )
        self.processed_image_publisher = self.create_publisher(
            Image, "watercolumn_processed_image", 10
        )

    def listener_callback(self, msg):
        self.message_counter += 1
        self.get_logger().info(
            "Received watercolumn_raw_image message #{}".format(self.message_counter)
        )
        processed_img = self.convert_image_to_polar(msg)
        self.get_logger().info(
            "Publishing processed image to {}".format(
                self.processed_image_publisher.topic
            )
        )
        self.processed_image_publisher.publish(processed_img)

    def convert_image_to_polar(self, msg):
        num_samples = msg.num_samples
        num_beams = msg.num_beams
        aperture = msg.swath_open
        pixel_values = msg.watercolumn_raw.data
        angles = msg.watercolumn_beam_directions

        # add more data to cover 360 degrees
        angle_per_beam = aperture / num_beams
        full_circle_beams = int(360 / angle_per_beam)
        full_circle_image = np.zeros((num_samples, full_circle_beams))
        # populate middle part with the original data
        start_index = int((full_circle_beams - num_beams) / 2)
        full_circle_image[:, start_index : start_index + num_beams] = pixel_values

        full_circle_image = full_circle_image.T
        img_height, img_width = 512, 512
        cartesian = cv2.warpPolar(
            full_circle_image,
            (img_width, img_height),
            (img_width / 2, img_height / 2),
            img_width / 2,
            cv2.WARP_INVERSE_MAP+cv2.INTER_LINEAR+cv2.WARP_FILL_OUTLIERS,
        )

        cartesian_image = Image()
        cartesian_image.data = cartesian
        cartesian_image.width = img_width
        cartesian_image.height = img_height
        cartesian_image.encoding = "mono8"
        cartesian_image.is_bigendian = False
        cartesian_image.step = num_samples*2
        return cartesian_image


def main(args=None):
    rclpy.init(args=args)
    watercolumn_visualizer = WaterColumnVisualizer()
    rclpy.spin(watercolumn_visualizer)
    watercolumn_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
