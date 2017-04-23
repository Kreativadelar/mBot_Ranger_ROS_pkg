# mBot_Ranger_ROS_pkg
Makeblock Me Auriga ROS package.

Features:
--
- Get ultrasonic sensor data - Topic 
- Move robot - Service (rosservice call /makeblock_ros_move_motors -- 0 0)

Hardware:
--
Connect ultrasonic sensor to port 9

Required:
--
- pip install megapi
- pip install pySerial

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

