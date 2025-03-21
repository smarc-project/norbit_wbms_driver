import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import socket
import array

import rclpy.parameter

class WBMSDriverNode(Node):
    """
    Class to configure settings of the WBMS driver.
    """

    def __init__(self):
        super().__init__('wbms_driver')
        self.get_logger().info("Starting WBMS driver node.")
        self.sonar_ip = '127.0.0.1' #TODO: modify in launch file
        self.sonar_port = 2209

        self.declare_parameters(
            namespace='',
            parameters=[
                # SONAR SETTINGS
                ('set_mode', -1),
                ('set_sidescan_model', -1),
                ('set_flip', -1),
                ('set_gate_mode', -1),
                ('set_gate_tilt', -1),
                ('set_autogate_preset', -1),
                # RANGE SETTINGS
                ('set_range', [-1]),
                # SWATH SETTINGS
                ('set_direction', -1),
                ('set_opening_angle', -1),
                ('set_resolution', [-1]),
                ('set_mf_decimation', -1),
                # TX SETTINGS
                ('set_tx', [-1]),
                ('set_gain', -1),
                ('set_vga', -1),
                ('set_rate',-1),
                ('set_tp_rate', -1),
                ('set_snippet_rate', -1),
                ('set_trigger_mode', -1),
                # ENVIRONMENT SETTINGS
                ('sound_velocity', -1),
                ('set_time_source', -1),
                ('set_ntp_server', ''),
                ('set_power', -1)
        ])
        self.add_on_set_parameters_callback(self.send_parameters_update)


    def send_parameters_update(self, params):
        messages = []
        for param in params:
            if not isinstance(param.value, array.array):
                messages.append(f'{param.name} {param.value}')
            else:
                values_str = [str(param.value[i]) for i in range(len(param.value))]
                messages.append(f'{param.name} {" ".join(values_str)}')

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.sonar_ip, self.sonar_port))
            for m in messages:
                self.get_logger().info(f'Sending message: {m}')
                s.send(m.encode())
                reply = str(s.recv(1024).decode('utf-8'))
                self.get_logger().info(f'Received: {reply}')

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    wbms_driver = WBMSDriverNode()
    rclpy.spin(wbms_driver)
    wbms_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()