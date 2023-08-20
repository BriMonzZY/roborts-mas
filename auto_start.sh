#!/bin/bash

echo "123456" | sudo -S chmod 777 /dev/ttyACM0
echo "123456" | sudo -S chmod 777 /dev/ttyUSB0


# base
source ~/roborts_mas/devel/setup.bash 
{
    gnome-terminal -t "start1" -- bash -c "roslaunch roborts_bringup base.launch"
}
sleep 2s

# us_cam
source ~/roborts_mas/devel/setup.bash 
{
    gnome-terminal -t "start1" -- bash -c "rosrun roborts_camera roborts_camera_node"
}
sleep 2s

# # us_cam
# source ~/roborts_mas/devel/setup.bash 
# {
#     gnome-terminal -t "start1" -- bash -c "roslaunch roborts_bringup usb_cam.launch"
# }
# sleep 2s


# armor_detection
source ~/roborts_mas/devel/setup.bash 
{
    gnome-terminal -t "start1" -- bash -c "roslaunch roborts_bringup armor_detection.launch"
}
sleep 2s

# lidar
source ~/roborts_mas/devel/setup.bash
{
    gnome-terminal -t "start2" -- bash -c "roslaunch lslidar_x10_driver lslidar_x10_serial.launch"
}
sleep 2s

# # gmapping
# source ~/roborts_mas/devel/setup.bash
# {
#     gnome-terminal -t "start3" -- bash -c "roslaunch lslidar_x10_driver gmapping_demo.launch"
# }
# sleep 2s

# roborts
source ~/roborts_mas/devel/setup.bash 
{
gnome-terminal -t "start5" -- bash -c "roslaunch roborts_bringup roborts.launch"
}
sleep 2s

# decision
source ~/roborts_mas/devel/setup.bash 
{
gnome-terminal -t "decision block" -- bash -c "rosrun roborts_decision behavior_test_node"
}
sleep 2s
