#!/usr/bin/env python3
"""
waypoint_navigator_pd_sim.py  (self-contained)

PD waypoint navigator for the Husky in Isaac Sim, using Isaac ground-truth
odometry for localization.

Self-contained: the three helpers that used to live in robot_math_utils
(get_yaw_from_quaternion, normalize_angle_radians, distance_2d) are included
inline, so this file has no local-module import and can be dropped into a
fresh package as-is.

Integration contract with Isaac Sim:
  - Subscribes: nav_msgs/Odometry on /odom
      (Isaac "ROS2 Publish Odometry" fed by "Isaac Compute Odometry Node"
       whose chassisPrim = /husky/base_link)
  - Publishes:  geometry_msgs/Twist on /cmd_vel
      (must match the ROS2 Subscribe Twist "Topic Name" in the action graph)

Waypoints:
  - Reads waypoints.csv from the current working directory.
  - One waypoint per line: x,y,theta   (meters, meters, radians)
  - Lines starting with # are ignored.
  - Coordinates are in the /odom frame (origin = robot's start pose in Isaac).
"""

import math
import os
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String as StringMsg


# ---------------------------------------------------------------------------
# Inlined math helpers (were robot_math_utils)
# ---------------------------------------------------------------------------
def get_yaw_from_quaternion(q) -> float:
    """Yaw (rotation about Z) in radians from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle_radians(angle: float) -> float:
    """Wrap an angle to the range (-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    """Euclidean distance in the XY plane."""
    return math.hypot(x2 - x1, y2 - y1)


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
WAYPOINTS_FILENAME = "waypoints.csv"
CMD_VEL_TOPIC = "/cmd_vel"
ODOM_TOPIC = "/odom"
DEBUG_POSE = False  # set True to print pose ~1 Hz


class WaypointNavigatorPD(Node):
    def __init__(self):
        super().__init__('waypoint_navigator_pd_node')
        self.get_logger().info('PD Waypoint Navigator initializing (Isaac Sim / odometry)...')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.status_publisher = self.create_publisher(StringMsg, '/navigation_status', 10)

        # Subscriber — Isaac ground-truth odometry
        self.pose_subscriber = self.create_subscription(
            Odometry, ODOM_TOPIC, self.pose_callback, 10
        )
        self.current_robot_x = 0.0
        self.current_robot_y = 0.0
        self.current_robot_yaw = 0.0
        self.pose_is_initialized = False
        self._last_debug_time = 0.0

        # Waypoint data
        self.target_waypoints = []
        self.current_waypoint_index = -1

        # --- PD Controller & Navigation Parameters (TUNE THESE) ---
        self.goal_xy_tolerance = 0.15
        self.goal_final_theta_tolerance = 0.20
        self.approach_heading_tolerance = 0.5

        self.max_linear_speed = 0.15
        self.max_angular_speed = 0.30

        self.linear_Kp = 0.5
        self.angular_Kp = 0.5
        self.linear_Kd = 0.15
        self.angular_Kd = 0.3

        self.previous_distance_error = 0.0
        self.previous_heading_error_to_point = 0.0
        self.previous_final_heading_error = 0.0
        self.last_time_error_calculated = self.get_clock().now().nanoseconds / 1e9

        # Navigation States
        self.NAV_STATE_IDLE = "IDLE"
        self.NAV_STATE_LOADING_WAYPOINTS = "LOADING_WAYPOINTS"
        self.NAV_STATE_ALIGNING_TO_POINT = "ALIGNING_TO_POINT"
        self.NAV_STATE_MOVING_TO_POINT = "MOVING_TO_POINT"
        self.NAV_STATE_REACHED_POINT_XY = "REACHED_POINT_XY"
        self.NAV_STATE_ALIGNING_FINAL_ORIENTATION = "ALIGNING_FINAL_ORIENTATION"
        self.NAV_STATE_WAYPOINT_COMPLETE = "WAYPOINT_COMPLETE"
        self.NAV_STATE_ALL_WAYPOINTS_DONE = "ALL_WAYPOINTS_DONE"
        self.current_nav_state = self.NAV_STATE_LOADING_WAYPOINTS

        self.load_waypoints_from_file()

        if self.target_waypoints:
            self.current_waypoint_index = 0
            self.current_nav_state = self.NAV_STATE_ALIGNING_TO_POINT
            self.log_and_publish_status(
                f"Loaded {len(self.target_waypoints)} waypoints. Targeting waypoint 0."
            )
        else:
            self.current_nav_state = self.NAV_STATE_IDLE
            self.log_and_publish_status("No waypoints loaded. Navigator is idle.")

        self.control_loop_timer = self.create_timer(0.05, self.navigation_control_loop)
        self.get_logger().info('Node initialized. Control loop started.')

    def pose_callback(self, msg: Odometry):
        self.current_robot_x = msg.pose.pose.position.x
        self.current_robot_y = msg.pose.pose.position.y
        raw_yaw = get_yaw_from_quaternion(msg.pose.pose.orientation)
        self.current_robot_yaw = normalize_angle_radians(raw_yaw)

        if not self.pose_is_initialized:
            self.pose_is_initialized = True
            self.last_time_error_calculated = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(
                f"Initial robot pose: x={self.current_robot_x:.2f}, "
                f"y={self.current_robot_y:.2f}, yaw={self.current_robot_yaw:.2f}"
            )

        if DEBUG_POSE:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_debug_time > 1.0:
                self._last_debug_time = now
                self.get_logger().info(
                    f"[pose] x={self.current_robot_x:.2f} y={self.current_robot_y:.2f} "
                    f"yaw={self.current_robot_yaw:.2f}"
                )

    def load_waypoints_from_file(self):
        self.target_waypoints = []
        full_path = os.path.abspath(WAYPOINTS_FILENAME)
        if not os.path.exists(full_path):
            self.get_logger().warn(f"Waypoint file not found: {full_path}")
            return
        try:
            with open(WAYPOINTS_FILENAME, 'r') as f:
                for line_number, line in enumerate(f):
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue
                    try:
                        parts = line.split(',')
                        x = float(parts[0])
                        y = float(parts[1])
                        theta = normalize_angle_radians(float(parts[2]))
                        self.target_waypoints.append({'x': x, 'y': y, 'theta': theta})
                    except (IndexError, ValueError) as e:
                        self.get_logger().warn(
                            f"Skipping malformed line {line_number+1} in "
                            f"{WAYPOINTS_FILENAME}: '{line}'. Error: {e}"
                        )
            self.get_logger().info(f"Loaded {len(self.target_waypoints)} waypoints from {full_path}")
        except IOError as e:
            self.get_logger().error(f"Could not read waypoints from file {full_path}: {e}")

    def navigation_control_loop(self):
        if not self.pose_is_initialized:
            self.log_and_publish_status("Waiting for initial robot pose (is /odom publishing?)...")
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time_error_calculated
        if dt <= 0:
            dt = 1e-9
        self.last_time_error_calculated = current_time

        if self.current_nav_state == self.NAV_STATE_IDLE:
            self.stop_robot()
            return
        if self.current_nav_state == self.NAV_STATE_ALL_WAYPOINTS_DONE:
            self.stop_robot()
            return
        if self.current_waypoint_index < 0 or self.current_waypoint_index >= len(self.target_waypoints):
            self.log_and_publish_status("Error: Invalid waypoint index. Setting IDLE.")
            self.current_nav_state = self.NAV_STATE_IDLE
            self.stop_robot()
            return

        current_goal = self.target_waypoints[self.current_waypoint_index]
        goal_x = current_goal['x']
        goal_y = current_goal['y']
        goal_theta = current_goal['theta']

        distance_to_goal_xy = distance_2d(self.current_robot_x, self.current_robot_y, goal_x, goal_y)
        angle_to_goal_point = math.atan2(goal_y - self.current_robot_y, goal_x - self.current_robot_x)
        heading_error_to_point = normalize_angle_radians(angle_to_goal_point - self.current_robot_yaw)
        final_heading_error = normalize_angle_radians(goal_theta - self.current_robot_yaw)

        twist_cmd = Twist()

        if self.current_nav_state == self.NAV_STATE_ALIGNING_TO_POINT:
            self.log_and_publish_status(
                f"Wpt {self.current_waypoint_index}: Aligning to point. Err: {heading_error_to_point:.2f} rad"
            )
            if abs(heading_error_to_point) > self.approach_heading_tolerance:
                d = (heading_error_to_point - self.previous_heading_error_to_point) / dt
                twist_cmd.angular.z = (self.angular_Kp * heading_error_to_point) + (self.angular_Kd * d)
                twist_cmd.angular.z = max(min(twist_cmd.angular.z, self.max_angular_speed), -self.max_angular_speed)
            else:
                self.get_logger().info(f"Wpt {self.current_waypoint_index}: Alignment to point complete.")
                self.current_nav_state = self.NAV_STATE_MOVING_TO_POINT
                self.previous_distance_error = distance_to_goal_xy
                twist_cmd.angular.z = 0.0
            self.previous_heading_error_to_point = heading_error_to_point

        elif self.current_nav_state == self.NAV_STATE_MOVING_TO_POINT:
            self.log_and_publish_status(
                f"Wpt {self.current_waypoint_index}: Moving. Dist: {distance_to_goal_xy:.2f}m"
            )
            if distance_to_goal_xy > self.goal_xy_tolerance:
                if abs(heading_error_to_point) > self.approach_heading_tolerance * 1.5:
                    self.get_logger().info(f"Wpt {self.current_waypoint_index}: Re-aligning during move.")
                    self.current_nav_state = self.NAV_STATE_ALIGNING_TO_POINT
                    self.previous_heading_error_to_point = heading_error_to_point
                    self.stop_robot()
                    return

                twist_cmd.linear.x = self.linear_Kp * distance_to_goal_xy
                twist_cmd.linear.x = max(min(twist_cmd.linear.x, self.max_linear_speed), 0.0)

                d = (heading_error_to_point - self.previous_heading_error_to_point) / dt
                twist_cmd.angular.z = (self.angular_Kp * heading_error_to_point * 0.7) + (self.angular_Kd * d * 0.5)
                twist_cmd.angular.z = max(min(twist_cmd.angular.z, self.max_angular_speed * 0.7),
                                          -self.max_angular_speed * 0.7)
            else:
                self.get_logger().info(f"Wpt {self.current_waypoint_index}: Point (x,y) reached.")
                self.current_nav_state = self.NAV_STATE_REACHED_POINT_XY
                self.stop_robot()
                self.previous_final_heading_error = final_heading_error
                return
            self.previous_distance_error = distance_to_goal_xy
            self.previous_heading_error_to_point = heading_error_to_point

        elif self.current_nav_state == self.NAV_STATE_REACHED_POINT_XY:
            self.log_and_publish_status(f"Wpt {self.current_waypoint_index}: Reached (x,y). Final orientation next.")
            self.current_nav_state = self.NAV_STATE_ALIGNING_FINAL_ORIENTATION
            self.previous_final_heading_error = final_heading_error

        elif self.current_nav_state == self.NAV_STATE_ALIGNING_FINAL_ORIENTATION:
            self.log_and_publish_status(
                f"Wpt {self.current_waypoint_index}: Aligning final orientation. Err: {final_heading_error:.2f} rad"
            )
            if abs(final_heading_error) > self.goal_final_theta_tolerance:
                twist_cmd.linear.x = 0.0
                d = (final_heading_error - self.previous_final_heading_error) / dt
                twist_cmd.angular.z = (self.angular_Kp * final_heading_error) + (self.angular_Kd * d)
                twist_cmd.angular.z = max(min(twist_cmd.angular.z, self.max_angular_speed), -self.max_angular_speed)
            else:
                self.get_logger().info(f"Wpt {self.current_waypoint_index}: Final orientation complete.")
                self.current_nav_state = self.NAV_STATE_WAYPOINT_COMPLETE
                self.stop_robot()
                return
            self.previous_final_heading_error = final_heading_error

        elif self.current_nav_state == self.NAV_STATE_WAYPOINT_COMPLETE:
            self.log_and_publish_status(f"Waypoint {self.current_waypoint_index} fully completed.")
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.target_waypoints):
                self.current_nav_state = self.NAV_STATE_ALIGNING_TO_POINT
                self.previous_heading_error_to_point = 0.0
                self.previous_distance_error = 0.0
                self.log_and_publish_status(f"Targeting next waypoint {self.current_waypoint_index}.")
            else:
                self.log_and_publish_status("All waypoints successfully navigated!")
                self.current_nav_state = self.NAV_STATE_ALL_WAYPOINTS_DONE

        self.cmd_vel_publisher.publish(twist_cmd)

    def stop_robot(self):
        self.cmd_vel_publisher.publish(Twist())

    def log_and_publish_status(self, status_text: str):
        self.get_logger().info(status_text)
        status_msg = StringMsg()
        status_msg.data = (
            f"WptIdx: {self.current_waypoint_index}, "
            f"State: {self.current_nav_state}, Msg: {status_text}"
        )
        self.status_publisher.publish(status_msg)

    def on_shutdown(self):
        self.get_logger().info("PD Waypoint Navigator shutting down...")
        self.stop_robot()
        self.get_logger().info("Robot stopped. Shutdown complete.")


def main(args=None):
    rclpy.init(args=args if args is not None else sys.argv)
    navigator_node = WaypointNavigatorPD()
    try:
        rclpy.spin(navigator_node)
    except KeyboardInterrupt:
        navigator_node.get_logger().info('Keyboard interrupt, shutting down navigator.')
    finally:
        navigator_node.on_shutdown()
        if rclpy.ok():
            navigator_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
