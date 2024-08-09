import rclpy
from rclpy.node import Node
import numpy as np
import math
import json
from msgs.msg import Pose, Wrench
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from control.utils import pose_to_np, odometry_to_np, wrench_to_np
from pathlib import Path

class MissionWaypoints(Node):

    def __init__(self):
        super().__init__("mission_waypoints")

        # path = Path.cwd() / "src" / "guidance" / "data" / "mission_waypoints.json"
        # with open(path, "r") as f:
        #     self.missions_json = json.load(f)

        self._blue_arrow_pos = np.array([5.2, 0.6, -0.5]) # TODO, MODIFY BASED ON COURSE
        self._red_arrow_pos = np.array([5.2, -0.6, -0.5])
        self._buoy_pos = np.array([9.3, -1.34, -0.5]) # ALPHA COURSE
        # self._buoy_pos = np.array([9.3, -1.34, -0.5]) # BETA COURSE
        # self._buoy_pos = np.array([9.3, -1.34, -0.5]) # C COURSE
        # self._buoy_pos = np.array([9.3, 0.0, -1.5, 0.0, 0.0, 0.0]) # DELTA COURSE

        self._tasks = np.array(["submerge", "move_through_gate_blue_arrow", "spin_ccw", "move_towards_buoy", "circumnavigate_buoy_ccw", "surface"]) # PERCEPTION TO CHANGE THESE
        
        self._hold_depth = -1.3
        self._missions = self.construct_waypoints({}, self._tasks, self._hold_depth, self._blue_arrow_pos, self._red_arrow_pos, self._buoy_pos, self)
        print(f"MISSIONS DICT: {self._missions}")

        # Indexes
        self._task_index = 0
        self._waypoints_index = 0
        self._waypoints = np.array(self._missions[self._tasks[self._task_index]], dtype=np.float64)

        self._desired_pose = self._waypoints[self._waypoints_index]
        self._desired_pose_pub = self.create_publisher(Pose, "desired_pose", 10)
        self._desired_wrench_sub = self.create_subscription(Wrench, "desired_wrench", self.wrench_callback, 10)

        self.dim_ = 6
        self.wrench = np.zeros(self.dim_)
        self.pose = np.zeros(self.dim_)
        self._current_state_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.current_state_callback, 10
        )

        timer_period = 0.1  # TODO: Don't hardcode this
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def construct_waypoints(self, missions, tasks, hold_depth, blue_arrow_pos, red_arrow_pos, buoy_pos, robot_pose):
        for task in tasks:
            waypoints_list = []
            if task == "submerge":
                point = [0.0, 0.0, hold_depth, 0.0, 0.0, robot_pose[5]]
                waypoints_list.append(point)
                point = [0.0, 0.0, hold_depth, 0.0, 0.0, 0.0]
                waypoints_list.append(point)
            elif task == "move_through_gate_blue_arrow":
                point = [blue_arrow_pos[0] + 1, blue_arrow_pos[1], hold_depth, 0.0, 0.0, 0.0] # Go 1m past gate
                waypoints_list.append(point)
            elif task == "move_through_gate_red_arrow":
                point = [red_arrow_pos[0] + 1, red_arrow_pos[1], hold_depth, 0.0, 0.0, 0.0] # Go 1m past gate
                waypoints_list.append(point)
            elif task == "spin_ccw":
                for i in range(8):
                    if "move_through_gate_blue_arrow" in tasks:
                        point = [blue_arrow_pos[0] + 1, blue_arrow_pos[1], hold_depth, 0.0, 0.0, np.pi * (i+1)]
                    elif "move_through_gate_red_arrow" in tasks:
                        point = [red_arrow_pos[0] + 1, red_arrow_pos[1], hold_depth, 0.0, 0.0, np.pi * (i+1)]
                    waypoints_list.append(point)
            elif task == "spin_cw":
                for i in range(8):
                    if "move_through_gate_blue_arrow" in tasks:
                        point = [blue_arrow_pos[0] + 1, blue_arrow_pos[1], hold_depth, 0.0, 0.0, -1 * np.pi * (i+1)]
                    elif "move_through_gate_red_arrow" in tasks:
                        point = [red_arrow_pos[0] + 1, red_arrow_pos[1], hold_depth, 0.0, 0.0, -1 * np.pi * (i+1)]
                    waypoints_list.append(point)
            elif task == "move_towards_buoy":
                point = [buoy_pos[0] - 1, buoy_pos[1], hold_depth, 0.0, 0.0, 0.0]
                waypoints_list.append(point)
            elif task == "circumnavigate_buoy_ccw":
                waypoints_list.append([buoy_pos[0] - 1, buoy_pos[1] - 1, hold_depth, 0.0, 0.0, 0.0])
                waypoints_list.append([buoy_pos[0] + 1, buoy_pos[1] - 1, hold_depth, 0.0, 0.0, 0.0])
                waypoints_list.append([buoy_pos[0] + 1, buoy_pos[1] + 1, hold_depth, 0.0, 0.0, 0.0])
                waypoints_list.append([buoy_pos[0] - 1, buoy_pos[1] + 1, hold_depth, 0.0, 0.0, 0.0])
                waypoints_list.append([buoy_pos[0] - 1, buoy_pos[1] - 1, hold_depth, 0.0, 0.0, 0.0])
            elif task == "circumnavigate_buoy_cw":
                waypoints_list.append([buoy_pos[0] - 1, buoy_pos[1] + 1, hold_depth, 0.0, 0.0, 0.0])
                waypoints_list.append([buoy_pos[0] + 1, buoy_pos[1] + 1, hold_depth, 0.0, 0.0, 0.0])
                waypoints_list.append([buoy_pos[0] + 1, buoy_pos[1] - 1, hold_depth, 0.0, 0.0, 0.0])
                waypoints_list.append([buoy_pos[0] - 1, buoy_pos[1] - 1, hold_depth, 0.0, 0.0, 0.0])
                waypoints_list.append([buoy_pos[0] - 1, buoy_pos[1] + 1, hold_depth, 0.0, 0.0, 0.0])
            elif task == "surface":
                waypoints_list.append([[buoy_pos[0] - 1, buoy_pos[1], -0.1, 0.0, 0.0, 0.0]])

            missions[task] = waypoints_list
        return missions

    def normalize_angle(
        self, angle
    ):  # TODO these are duplicate functions (originals in controls), fix later
        """Normalize the angle to be within the range of [-π, π]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def find_yaw_error(self, current_yaw, desired_yaw):  # TODO duplicate function
        """Find closest angle, handling pi -> -pi wrapping"""
        cur_error = self.normalize_angle(
            self.normalize_angle(desired_yaw) - self.normalize_angle(current_yaw)
        )
        sign = 1 if cur_error > 0 else -1
        wrapped_error = 2 * np.pi - np.abs(cur_error)

        if wrapped_error < cur_error:
            error = -1 * sign * wrapped_error
        else:
            error = cur_error

        return error

    def wrench_callback(self, msg: Wrench):
        self.wrench = wrench_to_np(msg)

    def timer_callback(self):
        msg = Pose(
            header=Header(stamp=self.get_clock().now().to_msg()),
            position=Vector3(
                x=self._desired_pose[0],
                y=self._desired_pose[1],
                z=self._desired_pose[2],
            ),
            orientation=Vector3(
                x=self._desired_pose[3],
                y=self._desired_pose[4],
                z=self._desired_pose[5],
            ),
        )
        # msg.data = pwm
        self._desired_pose_pub.publish(msg)

    def current_state_callback(self, msg: Odometry):
        self.pose = np.array(odometry_to_np(msg))
        eps_position = 0.5  # TODO tune
        eps_angle = 0.1  # TODO tune

        self._missions = self.construct_waypoints(missions=self._missions, tasks=["submerge"], buoy_pos=self._buoy_pos, blue_arrow_pos=self._blue_arrow_pos, red_arrow_pos=self._red_arrow_pos, robot_pose=self.pose)

        position_error = np.linalg.norm(self.pose[:3] - self._desired_pose[:3])
        yaw_error = np.abs(self.find_yaw_error(self.pose[5], self._desired_pose[5]))

        pose_rounded = [round(x, 2) for x in self.pose]
        wrench_rounded = [round(x, 2) for x in self.wrench]
        self.get_logger().info(
            f"\n\nCurrent Pose: {pose_rounded}\nDesired Pose: {self._desired_pose}\nPosition Error: {position_error}\nYaw Error: {yaw_error}\nWrench: {wrench_rounded}\nTask: {self._tasks[self._task_index]}"
        )

        if (yaw_error < eps_angle):# and (position_error < eps_position):
            if self._waypoints_index < len(self._waypoints) - 1: # Switch waypoints within a task
                self._waypoints_index += 1
                self._desired_pose = self._waypoints[self._waypoints_index]
            elif self._task_index < len(self._tasks) - 1: # Switch tasks
                self._task_index += 1
                self._waypoints_index = 0
                self._waypoints = np.array(self._missions[self._tasks[self._task_index]], dtype=np.float64)
            else:
                self.get_logger().info("END")


def main(args=None):
    rclpy.init(args=args)
    node = MissionWaypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
