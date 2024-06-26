from launch import LaunchDescription
from launch_ros.actions import Node

package = "control"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package=package,
            executable='thruster_manager',
        ),
        Node(
            package=package,
            executable='arduino',
        ),
    ])