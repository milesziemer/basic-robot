# basic-robot

Basic robot contoller using ROS2. Lidar node reads from RPLIDAR serial port, publishes to a navigation node. The navigation node decides the commands to deliver to the drive node, which uses the python gpiozero package to drive the motors.

### Robot Setup
 - RaspberryPi 4 running Ubuntu Server 20.04
 - Slamtec RPLIDAR A1M8 connected via USB cable 
 - 2 DC Motors wired up to the pi's GPIO pins via L293D motor driver 

## Credits
 - python3-gpiozero: interfacing with the gpio pins https://gpiozero.readthedocs.io/en/stable/
 - rplidar_ros: inspiration for setting up the serial port for rplidar https://github.com/robopeak/rplidar_ros