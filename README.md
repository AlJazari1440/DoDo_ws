# DoDo_ws 
This ROS workspace for the detection and the control of the robot "DoDo" to follow crop rows, uto setup the workspace, just download into home directory in Ubuntu operating system with ROS Kinetic, then type the command $catkin_make after the installation of the usb_cam package, if not installed.
Using this package inside your catkin_ws or The DoDo_ws built by the following instructions:
$ mkdir -p ~/DoDo_ws/src

Then copy the crop_row_follower folder inside the DoDo_ws/src directory 

$ cd ~/DoDo_ws/

$ catkin_make

$ cd devel

$ source setup.bash

  

*to launch the usb camera node 

$ roslaunch crop_row_follower usb_cam.launch video_device:=/dev/video0

*the row_tracker node

$ roslaunch crop_row_follower object_tracker.launch

*the crop_row_publisher node 

$ roslaunch crop_row_follower crop_row_publisher.launch
 
 
 
 
 have fun!
