from launch import LaunchDescription
from launch_ros.actions import Node
TOPIC = "control_test"

def generate_launch_description():
    pose_talker = Node(
        package = "ControlTest", 
        executable ="PosePublisher",
        name = "pose_talker", 
        parameters = [{
            "topic" : TOPIC
        }])
    pose_desried = Node(
        package = "ControlTest", 
        executable ="DesiredPublisher",
        name = "pose_talker", 
        parameters = [{
            "topic" : TOPIC
        }])
    
                       
