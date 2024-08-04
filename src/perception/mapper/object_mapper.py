import cv2
import numpy as np
from typing import Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

import message_filters
from cv_bridge import CvBridge
from control.ekf_object import ObjectEKF
from control.utils import odometry_to_np


from sensor_msgs.msg import Image
from yolov8_msgs.msg import BoundingBox2D
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray

from msgs.msg import Object
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3

class ObjectMapperNode(LifecycleNode):

    def __init__(self) -> None:
        super().__init__("object_mapper_node")

        self._class_to_color = {}
        self.cv_bridge = CvBridge()

        # params
        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().info("Object Mapper node created")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring {self.get_name()}")

        self.image_qos_profile = QoSProfile(
            reliability=self.get_parameter(
                "image_reliability").get_parameter_value().integer_value,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # TODO: final argument to ObjectEKF is not correct
        self.ekf_by_label = {
            'buoy': ObjectEKF(0, 0, 0),
            'red_arrow': ObjectEKF(0, 0, 0),
            'blue_arrow': ObjectEKF(0, 0, 0)
        }

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating {self.get_name()}")

        self.create_subscribers()
        self.create_publishers()

        return TransitionCallbackReturn.SUCCESS

    def create_subscribers(self):
        # Subscribe to image_raw topic, required for depth
        self.image_sub = message_filters.Subscriber(
            self, Image, "/oak/stereo/image_raw", qos_profile=self.image_qos_profile
        )

        # Subscribe to detections topic, required for 2d bounding box from sub perspective
        self.detections_sub = message_filters.Subscriber(
            self, DetectionArray, "detections", qos_profile=10
        )

        # Subscribe to state topic, required for converting coordinates from sub-context to global-context
        self.sub_state = message_filters.Subscriber(
            self, Odometry, "state", qos_profile=10
        )

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (self.image_sub, self.detections_sub, self.sub_state), 10, 0.5
        )

        self._synchronizer.registerCallback(self.detections_cb)

    def create_publishers(self):
        self._buoy_position_pub = self.create_publisher(Object, "buoy_position_topic", 10)
        self._red_arrow_pub = self.create_publisher(Object, "red_arrow_topic", 10)
        self._blue_arrow_pub = self.create_publisher(Object, "blue_arrow_topic", 10)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating {self.get_name()}")

        self.destroy_subscription(self.image_sub.sub)
        self.destroy_subscription(self.detections_sub.sub)
        self.destroy_subscription(self.detections_sub.sub)

        del self._synchronizer

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up {self.get_name()}")

        self.destroy_publisher(self._buoy_position_pub)
        self.destroy_publisher(self._red_arrow_pub)
        self.destroy_publisher(self._blue_arrow_pub)

        return TransitionCallbackReturn.SUCCESS

    def detections_cb(self, stereo_msg: Image, detection_msg: DetectionArray, sub_odometry: Odometry) -> None:
        """
        Description:
            This callback is invoked when the node receives published data from both the `image_raw` and
            `detections` topics. The callback is synchronized between these two topics so it fires on a definite
            rhythm.

        Plan:
            - Extract data from raw_image from to get object depth
            - Extract bounding box from detection to get object location in 2d
            - Combine readings to generate a 3d coordinate point in sub context
            - Convert coordinate from sub context to global context using subscription to Sub state
            - Run coordinate through KF and publish position and covariance matrix
        """

        depth_img = self.cv_bridge.imgmsg_to_cv2(stereo_msg)

        sub_position = odometry_to_np(sub_odometry)
        current_sub_position = sub_position[0:3] # First tuple is x,y,z position
        current_sub_orientation = sub_position[3:6] # Second tuple is orientation
        # TODO: Get sub's current dt
        current_sub_dt = 0

        detection: Detection
        for detection in detection_msg.detections:
            label = detection.class_name

            obj_ekf = self.ekf_by_label[label]
            obj_robot_position = self.get_local_position(detection, depth_img)

            # filter `update` call is handled by `handle_object_detection` implicitly
            obj_ekf.handle_object_detection(
                obj_robot_position,
                current_sub_position,
                current_sub_orientation
            )

            # filter updates prediction based on current velocity of sub
            obj_ekf.predict(current_sub_dt)

            position, velocity, covariance = obj_ekf.get_state()

            msg = Object(
                position=position,
                velocity=velocity,
                # flatten covariance matrix to row-major order
                covariance=covariance.flatten().tolist(),
                covariance_rows=covariance.shape[0],
                covariance_col=covariance.shape[1]
            )

            # TODO: not sure if this works in python, may need to use self.locals()[...]
            self[f'_{label.lower()}_topic'].publish(msg)

    def get_local_position(self, detection: Detection, depth_img: np.array) -> Vector3:
        bounding_box: BoundingBox2D = detection.bbox

        x = bounding_box.center.position.x
        y = bounding_box.center.position.y
        # TODO: Assume depth image is same 'size' as detection image, probably need to do some sort of normalization here
        # TODO: Depth in mm, probably need to convert (meters?)
        z = depth_img[x][y]

        return Vector3(x=x, y=y, z=z)


def main():
    rclpy.init()
    node = ObjectMapperNode()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()