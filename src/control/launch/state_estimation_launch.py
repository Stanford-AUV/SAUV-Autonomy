# state_estimation_launch.py
from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


# This should ONLY launch robot_localization pacakage!

def generate_launch_description():
    # base_link_to_imu = LaunchDescription([
    #     Node(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         output="screen",
    #         arguments=["0","0","0","0","0","0","base_link","imu_link",] # IMU is upside down and facing backwards
    #     )
    # ])

    # base_link_to_dvl = LaunchDescription([
    #     Node(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         output="screen",
    #         arguments=["0","0","0","0","0","0","base_link","dvl_link",] # warning: this MAY need to be rotated to match our coordinate system!
    #     )
    # ])

    # base_link_to_depth = LaunchDescription([
    #     Node(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         output="screen",
    #         arguments=["0","0","0","0","0","0","base_link","depth_link",] # warning: may need to be translated
    #     )
    # ])

    robot_loc_launch = LaunchDescription(
        [
            Node(
                package = "robot_localization",
                executable = "ekf_node",
                name = "ekf_filter_node",
                parameters = [
                    {
                        "world_frame": "odom",
                        "odom_frame": "odom",
                        "base_link_frame": "base_link",
                        "frequency": 120.0,
                        "imu0": "/imu/data", # IMU input -- would prefer to use pose!
                        "imu0_config": [False, False, False,
                                        False, False, False,
                                        False, False, False,
                                        True, True, True,
                                        True, True, True],
                        "imu0_differential": True,
                        "imu0_relative": False,
                        "imu0_queue_size": 10,
                        "imu0_remove_gravitational_acceleration": True,
                        "twist0": "/dvl/twist", # DVL input
                        "twist0_config": [False, False, False,
                                        False, False, False,
                                        True, True, True,
                                        False, False, False,
                                        False, False, False],
                        "twist0_differential": True,
                        "twist0_relative": False,
                        "pose0": "/depth/pose", # Depth input
                        "pose0_config": [False, False, True,
                                        False, False, False,
                                        False, False, False,
                                        False, False, False,
                                        False, False, False],
                        "pose0_relative": False,
                        "pose0_differential": True, 
                        "pose1": "/imu/pose",
                        "pose1_config": [False, False, False, # IMU pose
                                        True, True, True,
                                        False, False, False,
                                        False, False, False,
                                        False, False, False],
                        "pose1_relative": True,
                        "pose1_differential": False,
                    }
                ]
            )
        ]
    )

    return LaunchDescription(
        [
            # # launch_ros.actions.SetParameter(name='use_sim_time', value=True),
            # Node(
            #     package="xsens_mti_ros2_driver",
            #     executable="xsens_mti_node",
            #     name="xsens_mti_node",
            #     output="screen",
            # ),
            # Node(
            #     package="firmware",
            #     executable="depth_talker",
            #     name="depth_talker",
            #     output="screen",
            # ),
            # Node(
            #     package="dvl",
            #     executable="dvl_talker",
            #     name="dvl_talker",
            #     output="screen",
            # ),
            # Node(
            #     package="firmware",
            #     executable="sync_imu",
            #     name="sync_imu",
            #     output="screen",
            # ),
            # Node(
            #     package="firmware",
            #     executable="sync_filter",
            #     name="sync_filter",
            #     output="screen",
            # ),
            # Node(
            #     package="control",
            #     executable="state_estimator",
            #     name="state_estimator",
            #     output="screen",
            # ),
            # base_link_to_dvl,
            # base_link_to_imu,
            # base_link_to_depth,
            robot_loc_launch
        ]
    )
