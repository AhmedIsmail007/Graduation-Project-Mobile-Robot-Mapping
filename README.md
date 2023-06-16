# Graduation-Project-Mobile-Robot-Mapping
Running:

>> Raspberry Pi: 

$ roscore

$ rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0 _baud:=57600

$ roslaunch mybot_nav SLAM_REAL.launch # SLAM_REAL.launch for RPi


>> PC:

$ roslaunch mybot_nav SLAM_REAL.launch # SLAM_REAL.launch for PC
