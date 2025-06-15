from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
              '-configuration_directory', '/ros_ws/cartographer',
              '-configuration_basename', 'rosmaster_cartographer.lua'],
            remappings=[('scan', '/scan')]
        ),
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            remappings=[('scan', '/scan')]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_base_laser',
            arguments=['0','0','0','0','0','0','base_link','laser']
        )
    ])
