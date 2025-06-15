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
            package='driver_server',
            executable='driver_server',
            name='driver_server',
            output='screen'
        )
        Node(
            package='client_controller',
            executable='client_controller',
            name='client_controller',
            output='screen'
        )s
    ])
