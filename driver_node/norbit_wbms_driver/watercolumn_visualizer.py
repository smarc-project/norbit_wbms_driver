import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
from cv_bridge import CvBridge
import cv2

from sensor_msgs.msg import Image
from norbit_wbms_interfaces.msg import WaterColumn


class WaterColumnVisualizer(Node):
    def __init__(self):
        super().__init__("watercolumn_visualizer")
        self.get_logger().info("WaterColumnVisualizer node started")
        self.cvbridge = CvBridge()
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
        ranges = np.linspace(0, 1, num_samples)
        theta, r = np.meshgrid(angles, ranges)
        x = r * np.sin(theta)
        y = -r * np.cos(theta)

        fig, ax = plt.subplots(figsize=(6, 3), dpi=300)
        pixel_values = np.array(pixel_values).reshape(num_samples, num_beams)
        canvas = FigureCanvasAgg(fig)
        c = ax.pcolormesh(
            x, y, pixel_values, shading="auto", cmap="gray"
        )
        ax.set_aspect("equal")
        ax.axis("off")
        canvas.draw()
        image_np = np.frombuffer(canvas.tostring_rgb(), dtype=np.uint8)
        w, h = canvas.get_width_height()
        image_np = image_np.reshape((h, w, 3))
        cartesian_image = self.cvbridge.cv2_to_imgmsg(image_np, encoding="rgb8")
        return cartesian_image


def main(args=None):
    rclpy.init(args=args)
    watercolumn_visualizer = WaterColumnVisualizer()
    rclpy.spin(watercolumn_visualizer)
    watercolumn_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
