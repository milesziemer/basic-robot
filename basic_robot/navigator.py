#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from robot_interfaces.msg import DirectionRelative
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
        self.command = ""  # Keep track of previous command

    def log(self, info):
        """Wraps the logger"""
        self.get_logger().info(info)

    def get_command(self, theta: float, radius: float):
        """Parses the lidar message into a directional command"""
        msg = DirectionRelative()
        msg.forward = False
        msg.backward = False
        msg.left = False
        msg.right = False
        direction = "forward"  # direction command
        # Check if in warning distance
        if 0 < radius < WARN_DIST:
            if 135 < theta < 225:  # Danger front, go back
                self.log("Danger front, go back")
                msg.backward = True
                direction = "backward"
            elif 225 < theta < 315:  # Danger right, go left
                self.log("Danger right, go left")
                msg.left = True
                direction = "left"
            elif 45 < theta < 135:  # Danger left, go right
                self.log("Danger left, go right")
                msg.right = True
                direction = "right"
            else:
                self.log("Go forward")
                msg.forward = True
                direction = "forward"
        # Go forward otherwise
        else:
            self.log("Go forward")
            msg.forward = True
            direction = "forward"
        return msg, direction

    def callback(self, msg: Polar):
        """Runs on each lidar message. Gets the drive message/command, sends and/or records it"""
        msg_send, direction = self.get_command(msg.theta, msg.radius)
        # If the command has changed, send it
        if direction != self.command:
            self.pub.publish(msg=msg_send)
        self.command = (
            direction  # Record the command to compare when the next one comes
        )


def main(args=None):
    rclpy.init(args=args)

    navigator = Navigator()

    rclpy.spin(navigator)

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
