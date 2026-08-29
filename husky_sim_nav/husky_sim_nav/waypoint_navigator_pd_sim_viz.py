#!/usr/bin/env python3
"""
waypoint_navigator_pd_sim_viz.py

PD waypoint navigator for the Husky in Isaac Sim (ground-truth /odom),
WITH RViz2 visualization so you can SEE the waypoints and the robot's path.

What it publishes for visualization (all in the 'odom' frame):
  /waypoint_markers  (visualization_msgs/MarkerArray)
      - a numbered sphere at each waypoint
      - the CURRENT target waypoint highlighted (larger, yellow)
      - an arrow at each waypoint showing its target heading (theta)
  /robot_path        (nav_msgs/Path)
      - the trail the robot has actually driven (from /odom)

How to view in RViz2:
  ros2 run rviz2 rviz2
  - Set 'Fixed Frame' (Global Options) to: odom
  - Add -> By topic -> /waypoint_markers  (MarkerArray)
  - Add -> By topic -> /robot_path        (Path)
  - Add -> By topic -> /odom              (Odometry)  [shows the robot pose]

Control interface (unchanged):
  Subscribes nav_msgs/Odometry on /odom, publishes geometry_msgs/Twist on /cmd_vel.

Waypoints: waypoints.csv in the working directory, one 'x,y,theta' per line.
"""

import math
import os
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String as StringMsg
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------
def get_yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle_radians(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def distance_2d(x1, y1, x2, y2) -> float:
    return math.hypot(x2 - x1, y2 - y1)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
WAYPOINTS_FILENAME = "waypoints.csv"
CMD_VEL_TOPIC = "/cmd_vel"
ODOM_TOPIC = "/odom"
FRAME_ID = "odom"           # markers/path drawn in this frame; match RViz Fixed Frame
DEBUG_POSE = False


class WaypointNavigatorPD(Node):
    def __init__(self):
        super().__init__('waypoint_navigator_pd_node')
        self.get_logger().info('PD Waypoint Navigator (with RViz viz) initializing...')

        # Control publishers/subscriber
        self.cmd_vel_publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.status_publisher = self.create_publisher(StringMsg, '/navigation_status', 10)
        self.pose_subscriber = self.create_subscription(Odometry, ODOM_TOPIC, self.pose_callback, 10)

        # Visualization publishers
        self.marker_publisher = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.path_publisher = self.create_publisher(Path, '/robot_path', 10)
        self.robot_path = Path()
        self.robot_path.header.frame_id = FRAME_ID

        # State
        self.current_robot_x = 0.0
        self.current_robot_y = 0.0
        self.current_robot_yaw = 0.0
        self.pose_is_initialized = False
        self._last_debug_time = 0.0

        self.target_waypoints = []
        self.current_waypoint_index = -1

        # --- PD params ---
        self.goal_xy_tolerance = 0.15
        self.goal_final_theta_tolerance = 0.20
        self.approach_heading_tolerance = 0.3
        self.max_linear_speed = 0.15
        self.max_angular_speed = 0.30
        self.linear_Kp = 0.5
        self.angular_Kp = 0.9
        self.linear_Kd = 0.15
        self.angular_Kd = 0.1

        self.previous_distance_error = 0.0
        self.previous_heading_error_to_point = 0.0
        self.previous_final_heading_error = 0.0
        self.last_time_error_calculated = self.get_clock().now().nanoseconds / 1e9

        # States
        self.NAV_STATE_IDLE = "IDLE"
        self.NAV_STATE_ALIGNING_TO_POINT = "ALIGNING_TO_POINT"
        self.NAV_STATE_MOVING_TO_POINT = "MOVING_TO_POINT"
        self.NAV_STATE_REACHED_POINT_XY = "REACHED_POINT_XY"
        self.NAV_STATE_ALIGNING_FINAL_ORIENTATION = "ALIGNING_FINAL_ORIENTATION"
        self.NAV_STATE_WAYPOINT_COMPLETE = "WAYPOINT_COMPLETE"
        self.NAV_STATE_ALL_WAYPOINTS_DONE = "ALL_WAYPOINTS_DONE"
        self.current_nav_state = self.NAV_STATE_IDLE

        self.load_waypoints_from_file()

        if self.target_waypoints:
            self.current_waypoint_index = 0
            self.current_nav_state = self.NAV_STATE_ALIGNING_TO_POINT
            self.get_logger().info(
                f"Loaded {len(self.target_waypoints)} waypoints. First target: "
                f"({self.target_waypoints[0]['x']:.2f}, {self.target_waypoints[0]['y']:.2f})"
            )
        else:
            self.current_nav_state = self.NAV_STATE_IDLE
            self.get_logger().warn("No waypoints loaded. Navigator is idle. Is waypoints.csv in the working directory?")

        self.control_loop_timer = self.create_timer(0.05, self.navigation_control_loop)  # 20 Hz
        # Publish markers at a steady 2 Hz so RViz always has them even before motion
        self.marker_timer = self.create_timer(0.5, self.publish_waypoint_markers)
        self.get_logger().info('Node initialized. Control loop + viz started.')

    # ----------------------------------------------------------------------
    # Odometry
    # ----------------------------------------------------------------------
    def pose_callback(self, msg: Odometry):
        self.current_robot_x = msg.pose.pose.position.x
        self.current_robot_y = msg.pose.pose.position.y
        self.current_robot_yaw = normalize_angle_radians(get_yaw_from_quaternion(msg.pose.pose.orientation))

        if not self.pose_is_initialized:
            self.pose_is_initialized = True
            self.last_time_error_calculated = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(
                f"Initial robot pose: x={self.current_robot_x:.2f}, "
                f"y={self.current_robot_y:.2f}, yaw={math.degrees(self.current_robot_yaw):.1f} deg"
            )

        # Append to the traveled path for RViz
        ps = PoseStamped()
        ps.header.frame_id = FRAME_ID
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = msg.pose.pose
        self.robot_path.poses.append(ps)
        if len(self.robot_path.poses) > 2000:          # cap memory
            self.robot_path.poses = self.robot_path.poses[-2000:]
        self.robot_path.header.stamp = ps.header.stamp
        self.path_publisher.publish(self.robot_path)

        if DEBUG_POSE:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_debug_time > 1.0:
                self._last_debug_time = now
                self.get_logger().info(
                    f"[pose] x={self.current_robot_x:.2f} y={self.current_robot_y:.2f} "
                    f"yaw={math.degrees(self.current_robot_yaw):.1f}"
                )

    # ----------------------------------------------------------------------
    # Waypoints
    # ----------------------------------------------------------------------
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
                        x = float(parts[0]); y = float(parts[1])
                        theta = normalize_angle_radians(float(parts[2]))
                        self.target_waypoints.append({'x': x, 'y': y, 'theta': theta})
                    except (IndexError, ValueError) as e:
                        self.get_logger().warn(f"Skipping malformed line {line_number+1}: '{line}'. {e}")
            self.get_logger().info(f"Loaded {len(self.target_waypoints)} waypoints from {full_path}")
        except IOError as e:
            self.get_logger().error(f"Could not read {full_path}: {e}")

    # ----------------------------------------------------------------------
    # Visualization: waypoint markers
    # ----------------------------------------------------------------------
    def publish_waypoint_markers(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, wp in enumerate(self.target_waypoints):
            is_current = (i == self.current_waypoint_index)

            # Sphere at the waypoint
            m = Marker()
            m.header.frame_id = FRAME_ID
            m.header.stamp = now
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=wp['x'], y=wp['y'], z=0.1)
            m.pose.orientation.w = 1.0
            size = 0.35 if is_current else 0.22
            m.scale.x = m.scale.y = m.scale.z = size
            if is_current:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.9, 0.1, 1.0   # yellow = current
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.5, 1.0, 0.85   # blue = pending/done
            arr.markers.append(m)

            # Text label with the waypoint number
            t = Marker()
            t.header.frame_id = FRAME_ID
            t.header.stamp = now
            t.ns = "waypoint_labels"
            t.id = 1000 + i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position = Point(x=wp['x'], y=wp['y'], z=0.5)
            t.pose.orientation.w = 1.0
            t.scale.z = 0.3
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = str(i)
            arr.markers.append(t)

            # Arrow showing the target heading (theta) at the waypoint
            a = Marker()
            a.header.frame_id = FRAME_ID
            a.header.stamp = now
            a.ns = "waypoint_headings"
            a.id = 2000 + i
            a.type = Marker.ARROW
            a.action = Marker.ADD
            a.pose.position = Point(x=wp['x'], y=wp['y'], z=0.1)
            a.pose.orientation = yaw_to_quaternion(wp['theta'])
            a.scale.x = 0.5   # length
            a.scale.y = 0.06  # width
            a.scale.z = 0.06  # height
            a.color.r, a.color.g, a.color.b, a.color.a = 0.2, 1.0, 0.3, 0.9
            arr.markers.append(a)

        self.marker_publisher.publish(arr)

    # ----------------------------------------------------------------------
    # Control loop
    # ----------------------------------------------------------------------
    def navigation_control_loop(self):
        if not self.pose_is_initialized:
            self.log_and_publish_status("Waiting for initial robot pose (is /odom publishing? is the sim playing?)...")
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time_error_calculated
        if dt <= 0:
            dt = 1e-9
        self.last_time_error_calculated = current_time

        if self.current_nav_state in (self.NAV_STATE_IDLE, self.NAV_STATE_ALL_WAYPOINTS_DONE):
            self.stop_robot()
            return
        if not (0 <= self.current_waypoint_index < len(self.target_waypoints)):
            self.log_and_publish_status("Invalid waypoint index -> IDLE.")
            self.current_nav_state = self.NAV_STATE_IDLE
            self.stop_robot()
            return

        goal = self.target_waypoints[self.current_waypoint_index]
        gx, gy, gtheta = goal['x'], goal['y'], goal['theta']

        dist = distance_2d(self.current_robot_x, self.current_robot_y, gx, gy)
        angle_to_point = math.atan2(gy - self.current_robot_y, gx - self.current_robot_x)
        heading_err = normalize_angle_radians(angle_to_point - self.current_robot_yaw)
        final_heading_err = normalize_angle_radians(gtheta - self.current_robot_yaw)

        cmd = Twist()

        if self.current_nav_state == self.NAV_STATE_ALIGNING_TO_POINT:
            # Clearer, throttled log: target, distance, heading error in degrees
            self.log_and_publish_status(
                f"WP{self.current_waypoint_index} ALIGN -> target ({gx:.2f},{gy:.2f}) "
                f"dist {dist:.2f}m heading_err {math.degrees(heading_err):.0f} deg"
            )
            if abs(heading_err) > self.approach_heading_tolerance:
                d = (heading_err - self.previous_heading_error_to_point) / dt
                cmd.angular.z = self.angular_Kp * heading_err + self.angular_Kd * d
                cmd.angular.z = max(min(cmd.angular.z, self.max_angular_speed), -self.max_angular_speed)
            else:
                self.get_logger().info(f"WP{self.current_waypoint_index}: aligned, moving in.")
                self.current_nav_state = self.NAV_STATE_MOVING_TO_POINT
                self.previous_distance_error = dist
                cmd.angular.z = 0.0
            self.previous_heading_error_to_point = heading_err

        elif self.current_nav_state == self.NAV_STATE_MOVING_TO_POINT:
            self.log_and_publish_status(
                f"WP{self.current_waypoint_index} MOVE  -> target ({gx:.2f},{gy:.2f}) dist {dist:.2f}m"
            )
            if dist > self.goal_xy_tolerance:
                if abs(heading_err) > self.approach_heading_tolerance * 1.5:
                    self.get_logger().info(f"WP{self.current_waypoint_index}: drifted, re-aligning.")
                    self.current_nav_state = self.NAV_STATE_ALIGNING_TO_POINT
                    self.previous_heading_error_to_point = heading_err
                    self.stop_robot()
                    return
                cmd.linear.x = max(min(self.linear_Kp * dist, self.max_linear_speed), 0.0)
                d = (heading_err - self.previous_heading_error_to_point) / dt
                cmd.angular.z = self.angular_Kp * heading_err * 0.7 + self.angular_Kd * d * 0.5
                cmd.angular.z = max(min(cmd.angular.z, self.max_angular_speed * 0.7), -self.max_angular_speed * 0.7)
            else:
                self.get_logger().info(f"WP{self.current_waypoint_index}: reached (x,y).")
                self.current_nav_state = self.NAV_STATE_REACHED_POINT_XY
                self.stop_robot()
                self.previous_final_heading_error = final_heading_err
                return
            self.previous_distance_error = dist
            self.previous_heading_error_to_point = heading_err

        elif self.current_nav_state == self.NAV_STATE_REACHED_POINT_XY:
            self.current_nav_state = self.NAV_STATE_ALIGNING_FINAL_ORIENTATION
            self.previous_final_heading_error = final_heading_err

        elif self.current_nav_state == self.NAV_STATE_ALIGNING_FINAL_ORIENTATION:
            self.log_and_publish_status(
                f"WP{self.current_waypoint_index} TURN  -> final heading, err {math.degrees(final_heading_err):.0f} deg"
            )
            if abs(final_heading_err) > self.goal_final_theta_tolerance:
                cmd.linear.x = 0.0
                d = (final_heading_err - self.previous_final_heading_error) / dt
                cmd.angular.z = self.angular_Kp * final_heading_err + self.angular_Kd * d
                cmd.angular.z = max(min(cmd.angular.z, self.max_angular_speed), -self.max_angular_speed)
            else:
                self.get_logger().info(f"WP{self.current_waypoint_index}: DONE.")
                self.current_nav_state = self.NAV_STATE_WAYPOINT_COMPLETE
                self.stop_robot()
                return
            self.previous_final_heading_error = final_heading_err

        elif self.current_nav_state == self.NAV_STATE_WAYPOINT_COMPLETE:
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.target_waypoints):
                nxt = self.target_waypoints[self.current_waypoint_index]
                self.current_nav_state = self.NAV_STATE_ALIGNING_TO_POINT
                self.previous_heading_error_to_point = 0.0
                self.previous_distance_error = 0.0
                self.get_logger().info(
                    f"--> Next: WP{self.current_waypoint_index} ({nxt['x']:.2f},{nxt['y']:.2f})"
                )
            else:
                self.get_logger().info("ALL WAYPOINTS COMPLETE.")
                self.current_nav_state = self.NAV_STATE_ALL_WAYPOINTS_DONE

        self.cmd_vel_publisher.publish(cmd)

    # ----------------------------------------------------------------------
    def stop_robot(self):
        self.cmd_vel_publisher.publish(Twist())

    def log_and_publish_status(self, text: str):
        self.get_logger().info(text)
        msg = StringMsg()
        msg.data = f"WP{self.current_waypoint_index} | {self.current_nav_state} | {text}"
        self.status_publisher.publish(msg)

    def on_shutdown(self):
        self.get_logger().info("Shutting down, stopping robot...")
        self.stop_robot()


def main(args=None):
    rclpy.init(args=args if args is not None else sys.argv)
    node = WaypointNavigatorPD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt.')
    finally:
        node.on_shutdown()
        if rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()