from launch import LaunchDescription
from launch_ros.actions import Node

package = "control"
ros_arguments = ["--log-level", "fatal", "-p", "use_sim_time:=true"]


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package=package,
                executable="gazebo_manager",
                ros_arguments=ros_arguments,
            ),
            Node(
                package="guidance",
                executable="waypoints",
                ros_arguments=ros_arguments,
            ),
            # Node(
            #     package=package,
            #     executable="state_estimator",
            #     # ros_arguments=["-p", "use_sim_time:=true"],
            #     ros_arguments=ros_arguments,
            # ),
            Node(
                package=package,
                executable="controller",
                # ros_arguments=ros_arguments,
            ),
            Node(
                package=package,
                executable="thruster_manager",
                ros_arguments=ros_arguments,
            ),
        ]
    )
