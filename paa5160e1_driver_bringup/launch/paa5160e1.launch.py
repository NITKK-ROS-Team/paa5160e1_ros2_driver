from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package='paa5160e1_driver_node',
        executable='paa5160e1_driver_node_exec',
        name='paa5160e1_driver_node',
        output='screen',
        parameters=[{
            'dev': '/dev/ttyACM0',
            'baudrate': 115200,
            'timeout_ms': 100,
            'spin_ms': 1,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'device_frame_id': 'device',
        }]
    )

    return LaunchDescription([node])
