# ROS2_PX4_Offboard_Example_WS
by Joe

## Description
First little offboard example I wrote for competiton. 
Creds to Jaeyoung Lim for codes. 

## So far
So far This code is able to do the following 
- Change drone to offboard mode
- Arm the drone
- Take off
- Follow the waypoint given in control.py

## Note
- Processes.py can launch QGC, launch MicroXRCE and launch gazebo with the drone but I have disabled them
- Modify the launch file to launch specific files 
- Waypoint works in SITL but not irl atm (subject to change)