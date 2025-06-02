from launch import LaunchDescription
from launch_ros.actions import Node
from lolo_msgs.msg import Topics


def generate_launch_description():
    ld = LaunchDescription()

    # sonar_ip = "192.168.1.53"
    sonar_ip = "127.0.0.1"

    settings_node = Node(
        package="norbit_wbms_driver",
        executable="wbms_driver",
        name="settinsgs_node",
        output="screen",
        parameters=[
            {
                "sonar_ip": sonar_ip,
                "sonar_port": 2209,
            }
        ],
    )

    bathy_node = Node(
        package="norbit_wbms_driver",
        executable="bathymetry_parser",
        name="bathymetry_parser",
        output="screen",
        parameters=[
            {
                "sonar_ip": sonar_ip,
                "bathy_port": 2210,
                "output_topic": Topics.MBES_BATHY_TOPIC,
            }
        ],
    )

    watercolumn_node = Node(
        package="norbit_wbms_driver",
        executable="watercolumn_parser",
        name="watercolumn_parser",
        output="screen",
        parameters=[
            {
                "sonar_ip": sonar_ip,
                "watercolumn_port": 2211,
                "output_topic": Topics.MBES_WC_TOPIC,
                "output_image_topic": Topics.MBES_IMAGE_TOPIC,
            }
        ],
    )

    ld.add_action(settings_node)
    ld.add_action(bathy_node)
    ld.add_action(watercolumn_node)

    return ld
