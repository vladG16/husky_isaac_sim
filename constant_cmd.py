#!/usr/bin/env python3
"""
constant_cmd.py

Publishes a constant Twist to /cmd_vel to drive the Husky in Isaac Sim.
This is the reusable version of:

    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.5}, angular: {z: 0.0}}" -r 20

Speed and turn rate are ROS parameters so you don't have to edit the file.

Examples:
    # drive straight at 0.5 m/s
    ros2 run husky_sim_test constant_cmd

    # drive straight at 0.3 m/s
    ros2 run husky_sim_test constant_cmd --ros-args -p linear:=0.3

    # turn in place
    ros2 run husky_sim_test constant_cmd --ros-args -p linear:=0.0 -p angular:=0.5

    # arc: forward + turn, on a different topic
    ros2 run husky_sim_test constant_cmd --ros-args \
        -p linear:=0.5 -p angular:=0.3 -p topic:=/cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ConstantCmd(Node):
    def __init__(self):
        super().__init__('constant_cmd')

        # Parameters (override at runtime with --ros-args -p name:=value)
        self.declare_parameter('topic', '/cmd_vel')
        self.declare_parameter('linear', 0.5)    # forward speed, m/s
        self.declare_parameter('angular', 0.0)   # yaw rate, rad/s
        self.declare_parameter('rate_hz', 20.0)  # publish rate

        topic = self.get_parameter('topic').value
        self.lin = self.get_parameter('linear').value
        self.ang = self.get_parameter('angular').value
        rate_hz = self.get_parameter('rate_hz').value

        self.pub = self.create_publisher(Twist, topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

        self.get_logger().info(
            f'Publishing Twist(linear.x={self.lin}, angular.z={self.ang}) '
            f'on "{topic}" at {rate_hz} Hz. Ctrl+C to stop.'
        )

    def tick(self):
        msg = Twist()
        msg.linear.x = float(self.lin)
        msg.angular.z = float(self.ang)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConstantCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send a stop command on the way out so the robot doesn't coast.
        stop = Twist()
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
