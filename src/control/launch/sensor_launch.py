# sensor_launch.py
from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    base_link_to_imu = LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["0","0","0","0","0","0","base_link","imu_link",] # IMU is upside down and facing backwards
        )
    ])

    base_link_to_dvl = LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["0","0","0","0","0","0","base_link","dvl_link",] # warning: this MAY need to be rotated to match our coordinate system!
        )
    ])

    base_link_to_depth = LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["0","0","0","0","0","0","base_link","depth_link",] # warning: may need to be translated
        )
    ])

    map_to_world= LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["0","0","0","0","0","0","world","map",] # IMU is upside down and facing backwards
        )
    ])

    odom_to_map = LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["0","0","0","0","0","0","map","odom",] # IMU is upside down and facing backwards
        )
    ])

    return LaunchDescription(
        [
            # launch_ros.actions.SetParameter(name='use_sim_time', value=True),
            Node(
                package="xsens_mti_ros2_driver",
                executable="xsens_mti_node",
                name="xsens_mti_node",
                output="screen",
            ),
            Node(
                package="firmware",
                executable="sync_filter_exact",
                name="sync_filter_exact",
                output="screen",
            ),
            # Node( # use this OR sync_filter_exact ONLY
            #     package="firmware",
            #     executable="imu_pose",
            #     name="imu_pose",
            #     output="screen",
            # ),
            Node(
                package="firmware",
                executable="depth_talker",
                name="depth_talker",
                output="screen",
            ),
            Node(
                package="dvl",
                executable="dvl_talker",
                name="dvl_talker",
                output="screen",
            ),
            base_link_to_dvl,
            # base_link_to_imu,
            base_link_to_depth,
            map_to_world,
            odom_to_map,
        ]
    )
