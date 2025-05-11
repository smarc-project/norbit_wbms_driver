import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, IntegerRange, ParameterType
from rclpy.parameter import Parameter
import socket

class WBMSDriverNode(Node):
    """
    Class to configure settings of the WBMS driver.
    """

    def __init__(self):
        super().__init__('wbms_driver')
        self.get_logger().info("Starting WBMS driver node.")
        # self.sonar_ip = '127.0.0.1' #TODO: modify in launch file
        self.sonar_ip = '192.168.1.53' #TODO: modify in launch file
        self.sonar_port = 2209

        self.default_int_param = -1
        self.default_array_param = [-1]

        self._declare_sonar_setting_params()
        self._declare_range_and_swath_params()
        self._declare_tx_params()
        self._declare_environment_params()
        self.add_on_set_parameters_callback(self.send_parameters_update)

    def _declare_sonar_setting_params(self):
        self.declare_parameter(
            name='set_mode', value=self.default_int_param,
            descriptor=ParameterDescriptor(
                name='set_mode',
                type=ParameterType.PARAMETER_INTEGER,
                description='Operation modes: 0=forward, 1=bathy, 2=bathy amp, 4=BF pass-through, '\
                            '5=edge enhance, 6=super res, 7=multi detect'),
        )
        self.declare_parameter(
            name='set_sidescan_model', value=self.default_int_param,
            descriptor=ParameterDescriptor(
                name='set_sidescan_mode',
                type=ParameterType.PARAMETER_INTEGER,
                description='Sidescan/snippet mode: 0=snippets, 1=sidescan, 9=cropped sidescan',
            )
        )
        self.declare_parameter(
            name='set_flip', value=self.default_int_param,
            descriptor=ParameterDescriptor(
                name='set_flip',
                type=ParameterType.PARAMETER_INTEGER,
                description='sonar projector: 0=FLS (upwards), 1=bathy (towards stern)',
                integer_range=[IntegerRange(from_value=0, to_value=1, step=1)]
            ),
        )
        self.declare_parameter(
            name='set_gate_mode', value=self.default_int_param,
            descriptor=ParameterDescriptor(
                name='set_gate_mode',
                type=ParameterType.PARAMETER_INTEGER,
                description='0=gate by range, 1=gate by depth, 2=gate by range&depth',
                integer_range=[IntegerRange(from_value=0, to_value=2, step=1)]
            )
        )
        self.declare_parameter(
            name='set_gate_tilt', value=self.default_int_param,
            descriptor=ParameterDescriptor(
                name='set_gate_tilt',
                type=ParameterType.PARAMETER_INTEGER,
                description='[deg] [-45, 45]',
                integer_range=[IntegerRange(from_value=-45, to_value=45, step=1)]
            )
        )
        self.declare_parameter(
            name='set_autogate_preset', value=self.default_int_param,
            descriptor=ParameterDescriptor(
                name='set_autogate_preset',
                type=ParameterType.PARAMETER_INTEGER,
                description='0=off, 1=wide, 2=normal, 3=narrow',
                integer_range=[IntegerRange(from_value=0, to_value=3, step=1)]
            )
        )

    def _declare_range_and_swath_params(self):
        self.declare_parameter(
            name='set_range', value=self.default_array_param,
            descriptor=ParameterDescriptor(
                name='set_range',
                type=ParameterType.PARAMETER_INTEGER_ARRAY,
                description=(
                    f'set MBES range\n'
                    f'if set_gate_mode=0/1: set_range=[start, stop] range/depth\n'
                    f'if set_gate_mode=2: set_range=[start_range, stop_range, min_depth, max_depth]')
            )
        )
        self.declare_parameter(
            name='set_direction', value=self.default_int_param,
            descriptor=ParameterDescriptor(
                name='set_direction',
                type=ParameterType.PARAMETER_INTEGER,
                description='set direction for MBES swath',
                integer_range=[IntegerRange(from_value=-90, to_value=90, step=1)]
            )
        )
        self.declare_parameter(
            name='set_opening_angle', value=self.default_int_param,
            descriptor=ParameterDescriptor(
                name='set_opening_angle',
                type=ParameterType.PARAMETER_INTEGER,
                description='set opening_angle for MBES swath',
                integer_range=[IntegerRange(from_value=0, to_value=360, step=1)]
            )
        )
        self.declare_parameter(
            name='set_resolution', value=self.default_array_param,
            descriptor=ParameterDescriptor(
                name='set_resolution',
                type=ParameterType.PARAMETER_INTEGER_ARRAY,
                description='water column resolution, [VRES 64-2048, HRES 64-512]'
            )
        )
        self.declare_parameter(
            name='set_mf_decimation', value=self.default_int_param,
            descriptor=ParameterDescriptor(
                name='set_mf_decimation',
                type=ParameterType.PARAMETER_INTEGER,
                description='set mf_decimation for MBES swath',
                integer_range=[IntegerRange(from_value=1, to_value=16, step=1)]
            )
        )

    def _declare_tx_params(self):
        self.declare_parameter(
            name='set_tx', value=self.default_array_param,
            descriptor=ParameterDescriptor(
                name='set_tx',
                type=ParameterType.PARAMETER_INTEGER_ARRAY,
                description=(
                    f'set_tx for MBES: [frequency, bandwidth, amplitude, length], '
                    f'ranges: frequency [200, 700] kHz, bandwidth [0, 80] kHz,'
                    f'amplitude [0, 15], length [10, 819] microseconds'
                )
            )
        )
        self.declare_parameter(
            name='set_gain', value=self.default_int_param,
            descriptor=ParameterDescriptor(
                name='set_gain',
                type=ParameterType.PARAMETER_INTEGER,
                description='[0, 100] dB, fixed VGA gain. VGA ramp must be 0 for this to work',
                integer_range=[IntegerRange(from_value=0, to_value=100, step=1)]
            )
        )
        self.declare_parameter(
            name='set_vga', value=self.default_int_param,
            descriptor=ParameterDescriptor(
                name='set_gain',
                type=ParameterType.PARAMETER_INTEGER,
                description='VGA ramp in dB/100m',
            )
        )
        self.declare_parameter(
            name='set_rate', value=self.default_int_param,
            descriptor=ParameterDescriptor(
                name='set_rate',
                type=ParameterType.PARAMETER_INTEGER,
                description='[-1, 60] Hz, Max allowable ping rate, -1=auto',
                integer_range=[IntegerRange(from_value=-1, to_value=60, step=1)]
            )
        )
        self.declare_parameter(
            name='set_tp_rate', value=self.default_int_param,
            descriptor=ParameterDescriptor(
                name='set_tp_rate',
                type=ParameterType.PARAMETER_INTEGER,
                description='[-1, 60] Hz, water column output rate',
                integer_range=[IntegerRange(from_value=-1, to_value=60, step=1)]
            )
        )
        self.declare_parameter(
            name='set_snippet_rate', value=self.default_int_param,
            descriptor=ParameterDescriptor(
                name='set_snippet_rate',
                type=ParameterType.PARAMETER_INTEGER,
                description='[-1, 60] Hz, snippet output rate',
                integer_range=[IntegerRange(from_value=-1, to_value=60, step=1)]
            )
        )
        self.declare_parameter(
            name='set_trigger_mode', value=self.default_int_param,
            descriptor=ParameterDescriptor(
                name='set_trigger_mode',
                type=ParameterType.PARAMETER_INTEGER,
                description=(
                    f'trigger modes:\n'
                    f'0=fixed rate, 1=fixed rate (time sync), 2=full range (max allowed by set range),'
                    f'3=adaptive (max allowed by measured range), 4=external trigger (rising edge),'
                    f'5=external trigger (falling edge)'),
                integer_range=[IntegerRange(from_value=0, to_value=5, step=1)]
            )
        )

    def _declare_environment_params(self):
        self.declare_parameter(
            name='sound_velocity', value=self.default_int_param,
            descriptor=ParameterDescriptor(
                name='sound_velocity',
                type=ParameterType.PARAMETER_INTEGER,
                description='sets fixed sound velocity (-1=use external probe on serial port)',
            )
        )
        self.declare_parameter(
            name='set_time_source', value=self.default_int_param,
            descriptor=ParameterDescriptor(
                name='set_time_source',
                type=ParameterType.PARAMETER_INTEGER,
                description=(
                    f'time source:\n'
                    f'0=IRIG-B 1=NTP 2=IRIG-B(inverted) 3=NTP+PPS(pos) 4=NTP+PPS(neg) '
                    f'5=ZDA+PPS(pos), 6=ZDA+PPS(neg), 7=ZDA, 8=free run(time since boot)'
                ),
                integer_range=[IntegerRange(from_value=0, to_value=8, step=1)],
            )
        )
        self.declare_parameter(
            name='set_ntp_server', value='',
            descriptor=ParameterDescriptor(
                name='set_ntp_server',
                type=ParameterType.PARAMETER_STRING,
                description='set NTP server IP address',
            )
        )
        self.declare_parameter(
            name='set_power', value=self.default_int_param,
            descriptor=ParameterDescriptor(
                name='set_power',
                type=ParameterType.PARAMETER_INTEGER,
                description='Turn the pinging on and off, 0=off, 1=on',
                integer_range=[IntegerRange(from_value=0, to_value=1, step=1)],
            )
        )


    def send_parameters_update(self, params):
        messages = []
        for param in params:
            if param.type_ == Parameter.Type.INTEGER_ARRAY:
                values_str = [str(param.value[i]) for i in range(len(param.value))]
                messages.append(f'{param.name} {" ".join(values_str)}')
            else:
                messages.append(f'{param.name} {param.value}')

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.sonar_ip, self.sonar_port))
            reply = str(s.recv(1024).decode('utf-8'))
            self.get_logger().info(f'Received welcome message: {reply}')
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