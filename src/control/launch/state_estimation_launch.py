# state_estimation_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xsens_mti_ros2_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node',
            output='screen'
        ),
        Node(
            package='firmware',
            executable='depth_talker',
            name='depth_talker',
            output='screen'
        ),
        Node(
            package='dvl',
            executable='dvl_talker',
            name='dvl_talker',
            output='screen'
        ),
        Node(
            package='firmware',
            executable='sync_imu',
            name='sync_imu',
            output='screen'
        ),
        Node(
            package='firmware',
            executable='sync_filter',
            name='sync_filter',
            output='screen'
        ),
        Node(
            package='control',
            executable='state_estimator',
            name='state_estimator',
            output='screen'
        )
    ])
