# drive_base
This a ROS2 node for a drivebase using the Roboclaw motor controllers made by [Basic Micro Motion Control](https://www.basicmicro.com/).

## Pre-Reqs
[Roboclaw user Manual](https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf)
Before you use this package you need to calibrate the velocity PID on the Roboclaw.  This will require the
installation of their calibration software [Motion Studio](https://downloads.basicmicro.com/software/BMStudio/setup.exe) (Windows only).

Settings this project uses that need to be set in motion studio:
Control Mode: packet serial
Serial Packet Address: Right: 128, Left: 129
Baudrate: 115200

Additionally check that your encoders readings and motor directions line up in the software.

## Usage
Just clone the repo into your colcon workspace. It contains the ROS node and the [motor controller driver written by bmegli](https://github.com/bmegli/roboclaw) with a few added functionalities for speed control.
```bash
cd <workspace>/src
git clone https://https://github.com/ccresta1386/drive_base.git
cd <workspace>
colcon build
```
The package assumes you will be using USB control, make sure that ROS will have access to the "/dev/USBttyACMX" ports that your roboclaws are connected to.


## Parameters
The launch file can be configure at the command line with arguments, by changing the value in the launch file or through the rosparam server.

|Parameter|Default|Definition|
|-----|----------|-------|
|encoder_scale|-2560|The encoder scale for used in calculation for ticks_to_m conversion, use this to calibrate odometry|
|debug|false|To print or not to print positional information for encoder calibration|
|wheel_radius|0.09915|Set this per your wheels|
|wheelbase_x|0.16312|Set this for your wheelbase measurement in the X direction|
|wheelbase_y|0.3458|Set this for your wheelbase measurement in the Y direction|
|omnidirectional|false|whether or not to use the omnidirectional driving|

## Topics
###Subscribed
/cmd_vel [(geometry_msgs/msg/Twist.msg)](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html)  
Velocity commands for the mobile base.
###Published
/wheel/odometry [(nav_msgs/msg/Odometry.msg)](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html)  
Odometry output from the mobile base. Note omnidirectional odometry is not currently implemented.

