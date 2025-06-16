from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_laser',
            output='screen',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser']
        ),
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom',
                'publish_tf': True,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'max_time_between_scans': 0.5,
                'freq': 5.0}],
        )
    ])