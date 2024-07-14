from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_dvl',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'dvl_link']
        ),
        # # DVL relay node
        # Node(
        #     package='dvl_relay',
        #     executable='dvl_relay_node',
        #     name='dvl_relay_node',
        #     output='screen'
        # ),
        # EKF node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/home/mk/SAUV/SAUV-Autonomy/src/control/ekf.yaml'],
            arguments=['--ros-args', '--log-level', 'debug']
        )
    ])
