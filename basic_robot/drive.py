#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node

from gpiozero import PWMOutputDevice, LED
from basic_robot_interfaces.msg import Wheels

LEFT_FWD = 5
LEFT_BWD = 13
RIGHT_FWD = 6
RIGHT_BWD = 19

ENABLE = 26

PWM_FREQUENCY = 1000


# class DifferentialDrive:
#     """Controls the forward and backward pins, and the enable pin"""

#     def __init__(self):
#         self.left_fwd = PWMOutputDevice(pin=LEFT_FWD, frequency=PWM_FREQUENCY)
#         self.right_fwd = PWMOutputDevice(pin=RIGHT_FWD, frequency=PWM_FREQUENCY)
#         self.left_bwd = PWMOutputDevice(pin=LEFT_BWD, frequency=PWM_FREQUENCY)
#         self.right_bwd = PWMOutputDevice(pin=RIGHT_BWD, frequency=PWM_FREQUENCY)
#         self.enable = LED(pin=ENABLE)
#         self.enable.on()  # Enable is active high connected to MOSFET, turn on to stop

#     def stop(self):
#         self.left_fwd.off()
#         self.right_fwd.off()
#         self.left_bwd.off()
#         self.right_bwd.off()

#     def drive(self):
#         pass


class Drive(Node):
    """Subscribes to the drive topic, and setups the gpio pins to drive the robot"""

    def __init__(self):
        super().__init__("drive_node")
        self.sub = self.create_subscription(Wheels, "drive_topic", self.callback, 10)
        self.left_fwd = PWMOutputDevice(pin=LEFT_FWD, frequency=PWM_FREQUENCY)
        self.right_fwd = PWMOutputDevice(pin=RIGHT_FWD, frequency=PWM_FREQUENCY)
        self.left_bwd = PWMOutputDevice(pin=LEFT_BWD, frequency=PWM_FREQUENCY)
        self.right_bwd = PWMOutputDevice(pin=RIGHT_BWD, frequency=PWM_FREQUENCY)
        self.enable = LED(pin=ENABLE)
        self.enable.on()  # Enable is active high connected to MOSFET, turn on to stop

    def log(self, info):
        self.get_logger().info(info)

    def stop(self):
        """Stop the motors"""
        self.left_fwd.off()
        self.right_fwd.off()
        self.left_bwd.off()
        self.right_bwd.off()

    def callback(self, msg: Wheels):
        """Runs on message from drive topic, delivers proper drive command to gpio pins"""
        self.log(f"l: {msg.backleft}, r: {msg.backright}")
        if msg.backleft == 0 and msg.backright == 0:
            self.enable.on()
            self.stop()
        else:
            if msg.backright < 0 and msg.backleft > 0:
                self.log("turn right")
                self.right_fwd.off()
                self.left_bwd.off()
                self.right_bwd.on()
                self.left_fwd.on()
            elif msg.backright > 0 and msg.backleft < 0:
                self.log("turn left")
                self.right_bwd.off()
                self.left_fwd.off()
                self.right_fwd.on()
                self.left_bwd.on()
            elif msg.backright < 0 and msg.backleft < 0:
                self.log("back")
                self.right_fwd.off()
                self.left_fwd.off()
                self.right_bwd.on()
                self.left_bwd.on()
            else:
                self.log("forward")
                self.right_bwd.off()
                self.left_bwd.off()
                self.right_fwd.on()
                self.left_fwd.on()
            self.enable.off()


def main(args=None):
    rclpy.init(args=args)

    drive = Drive()

    rclpy.spin(drive)

    drive.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
