#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node

from robot_interfaces.msg import DirectionRelative
from gpiozero import PWMOutputDevice, LED

LEFT_FWD = 5
LEFT_BWD = 13
RIGHT_FWD = 6
RIGHT_BWD = 19

ENABLE = 26

PWM_FREQUENCY = 1000


class DifferentialDrive:
    """Controls the forward and backward pins, and the enable pin"""

    def __init__(self):
        self.left_fwd = PWMOutputDevice(pin=LEFT_FWD, frequency=PWM_FREQUENCY)
        self.right_fwd = PWMOutputDevice(pin=RIGHT_FWD, frequency=PWM_FREQUENCY)
        self.left_bwd = PWMOutputDevice(pin=LEFT_BWD, frequency=PWM_FREQUENCY)
        self.right_bwd = PWMOutputDevice(pin=RIGHT_BWD, frequency=PWM_FREQUENCY)
        self.enable = LED(pin=ENABLE)
        self.enable.on()  # Enable is active high connected to MOSFET, turn on to stop

    def stop(self):
        self.left_fwd.off()
        self.right_fwd.off()
        self.left_bwd.off()
        self.right_bwd.off()

    def drive(self, translation: int, rotation: int):
        """Commands are simple, rotate left or right, go forward or back, or stop"""
        if translation == 0 and rotation == 0:
            self.enable.on()
            self.stop()
        else:
            if translation < 0:
                self.left_fwd.off()
                self.right_fwd.off()
                self.left_bwd.on()
                self.right_bwd.on()
            elif rotation < 0:
                self.left_fwd.off()
                self.right_bwd.off()
                self.left_bwd.on()
                self.right_fwd.on()
            elif rotation > 0:
                self.left_bwd.off()
                self.right_fwd.off()
                self.left_fwd.on()
                self.right_bwd.on()
            else:
                self.left_bwd.off()
                self.right_bwd.off()
                self.left_fwd.on()
                self.right_fwd.on()
            self.enable.off()


class Drive(Node):
    """Subscribes to the drive topic, and setups the gpio pins to drive the robot"""

    def __init__(self):
        super().__init__("drive_node")
        self.subscription = self.create_subscription(
            DirectionRelative, "drive_topic", self.callback, 10
        )
        self.differential_drive = DifferentialDrive()

    def callback(self, msg: DirectionRelative):
        """Runs on message from drive topic, delivers proper drive command to gpio pins"""
        translation = 0
        rotation = 0
        if msg.left:
            rotation = 1
        elif msg.right:
            rotation = -1
        elif msg.forward:
            translation = 1
        elif msg.backward:
            translation = -1
        self.differential_drive.drive(translation=translation, rotation=rotation)


def main(args=None):
    rclpy.init(args=args)

    drive = Drive()

    rclpy.spin(drive)

    drive.destroy_node()
    rclpy.shutdown()


def test_left(dd: DifferentialDrive):
    dd.drive(translation=0, rotation=-1)
    time.sleep(3)
    dd.stop()


def test_right(dd: DifferentialDrive):
    dd.drive(translation=0, rotation=1)
    time.sleep(3)
    dd.stop()


def test_fwd(dd: DifferentialDrive):
    dd.drive(translation=1, rotation=0)
    time.sleep(3)
    dd.stop()


def test_bwd(dd: DifferentialDrive):
    dd.drive(translation=-1, rotation=0)
    time.sleep(3)
    dd.stop()


def test():

    print("start test")
    ddrive = DifferentialDrive()

    print("test left...")
    test_left(ddrive)
    print("done.\n")

    print("test right...")
    test_right(ddrive)
    print("done.\n")

    print("test forward...")
    test_fwd(ddrive)
    print("done.\n")

    print("test back...")
    test_bwd(ddrive)
    print("done.\n")


if __name__ == "__main__":
    main()
