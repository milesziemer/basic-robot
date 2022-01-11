#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from basic_robot_interfaces.msg import Wheels, Polar

BOT_LENGTH = 230  # millimeters
WARN_DIST = BOT_LENGTH


class Navigator(Node):
    """
    Navigator node subscribes to the lidar topic, and publishes on drive.
    Publisher has no timer, sends whenever it gets lidar info when needed.
    """

    def __init__(self):
        super().__init__("navigator_node")
        self.sub = self.create_subscription(Polar, "lidar_topic", self.callback, 10)
        self.pub = self.create_publisher(Wheels, "drive_topic", 10)
        self.prev_cmd = ""  # Keep track of previous command

    def log(self, info):
        """Wraps the logger"""
        self.get_logger().info(info)

    def callback(self, msg: Polar):
        cmd = Wheels()
        cmd.backleft = 1.0
        cmd.backright = 1.0
        direction = "forward"
        theta = msg.theta
        # Check close to an object, not including 0 (lidar sends that to indicate invalid)
        if 0 < msg.radius < WARN_DIST:
            if 135 < theta < 225:  # Danger front
                direction = "backward"
                cmd.backright = 1.0
                cmd.backleft = 1.0
            elif 225 < theta < 315:  # Danger right
                direction = "left"
                cmd.backright = 1.0
                cmd.backleft = -1.0
            elif 45 < theta < 135:  # Danger left
                direction = "right"
                cmd.backright = -1.0
                cmd.backleft = 1.0

        if direction != self.prev_cmd:
            self.log(f"publishing {direction}")
            self.pub.publish(cmd)
        self.prev_cmd = direction


def main(args=None):
    rclpy.init(args=args)

    navigator = Navigator()

    rclpy.spin(navigator)

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
