# mBot_Ranger_ROS_pkg
Makeblock mBot Ranger / Me Auriga ROS package.

This package is based on the work of [to4dy/makeblock-ros](https://github.com/to4dy/makeblock-ros) 

Features:
--
- Get ultrasonic sensor data - Topic 
- Move robot - Service (rosservice call /makeblock_ros_move_motors -- 0 0)

Hardware:
--
The Me Auriga is connected with USB cable to a Raspberry Pi running Ubuntu Mate and ROS Kinetic

Connect ultrasonic sensor to port 9

Required:
--
- [https://github.com/Kreativadelar/PythonForMeAuriga](https://github.com/Kreativadelar/PythonForMeAuriga)
- pip install pySerial
- For teleop. with xbox 360 controller you ned [Joy](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick) 

Installation:
--
cd ~/catkin_ws/src

git clone ..

Troubleshooting
--
Using Raspberry Pi - Ubuntu Mate
Problem with permission on serial port

Add user to group dialout:
usermod -a -G dialout **[user]**
*(Need to logout before it will work)*

Quick fix:
sudo chmod 666 /dev/**[port]** *ex.ttyUSB0*

