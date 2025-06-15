from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Percorso alla mappa
    map_file = os.path.join(
        get_package_share_directory('tuo_package'),
        'maps',
        'tua_mappa.pgm'
    )
    
    return LaunchDescription([
	Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 115200,
                'frame_id': 'laser'
            }],
            output='log'
        )
	Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='async_slam_toolbox_node',
            parameters=[{
		'params-file': './slam_toolbox_online.yaml'
            }],
            output='log'
        )
	Node(
            package='mapping',
            executable='keyboard',
            name='keyboard',
            output='screen'
        )
    ])
