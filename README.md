# DoDo_ws 
This ROS workspace for the detection and the control of the robot "DoDo" to follow crop rows, to setup the workspace, just download into home directory in Ubuntu operating system with ROS Kinetic, then type the command $catkin_make after the installation of the usb_cam package, if not installed.
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

*the rosserial node 

$ rosrun rosserial_python serial_node.py /dev/ttyACM0

DoDo Autonmous crop sprayer demo https://youtu.be/9pp_zGtAmGk 



 _________________________________
 __________________________________
 
 For the Computer Vision algorithm:
 
 Data set testing demo https://youtu.be/tlmTV2QY2gE 
 
The algorithm implemented in the node crop_row_publisher in the source file """"""crop_row_follower/src/crop_row_publisher_2.py"""""", this node subscibes to the /camera/rgb/image_raw topic , then has the image_callback function which utilizes the process_image function. The process_image function converts the image to gray scale then skeltonize the image and extract the lines. After this the intesections with the image borders are calculated and publishing to the /roi topic.

Inside the process_image function in the crop_row_publisher node, the crop rows are obtained. For the middle two rows, the x-offset ,parameter published to the region of interset /roi topic is the x-coordinate of intersection of the leftmost row with image border, and as known, the top left corner is the conventinal origin. While the width parameter in the Region of Interset is the diffrence between the x-coordinates of the intersection of both the middle two crop rows and the image border.

The row_tracker node inside the file crop_row_follower/src/row_tracker.py Subscibes to the /roi and control the DoDo by the standard Twist message in ROS, through publishing to the /cmd_vel which communicates with the DoDo using rosserial.

For more information about the ROS publishers and subscirbers nodes:

http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29 
 
For disscusion about the DoDo robot in the ROS-A group, see this video:

https://youtu.be/meLUXcqFWhs 

Thank you,

Mahmoud Shoshah

 ![DoDo](/DoDo.jpeg)
 
 DoDo Agriculture robot


 
 have fun!
