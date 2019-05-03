#!/usr/bin/env python

""" In the name of Allah

    This code is from two parts

    *first part: cv_bridge_demo.py - Version 1.1 2013-12-20,
    that was created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved
   
    **second part:line_detect_2.py by Peter Nicholls 2016,

    ***Eddited by MTSH in 2019 to publish to the region of interst topic /roi to the detected crop lines

    This node uses cv_bridge to map a ROS image topic to the equivalent OpenCV image stream(s).
    

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
import sys
import cv2
import math

from sensor_msgs.msg import Image, CameraInfo, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

HOUGH_RHO = 2                      # Distance resolution of the accumulator in pixels
HOUGH_ANGLE = math.pi*4.0/180     # Angle resolution of the accumulator in radians
HOUGH_THRESH_MAX = 100             # Accumulator threshold parameter. Only those lines are returned that get enough votes
HOUGH_THRESH_MIN = 10
HOUGH_THRESH_INCR = 1

NUMBER_OF_ROWS = 4# how many crop rows to detect

THETA_SIM_THRESH = math.pi*(6.0/180)   # How similar two rows can be
RHO_SIM_THRESH = 8   # How similar two rows can be
ANGLE_THRESH = math.pi*(30.0/180) # How steep angles the crop rows can be in radians


class cvBridgeDemo():
    def __init__(self):
        self.node_name = "crop_row_publisher"
        
        rospy.init_node(self.node_name)
        
        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)
        
        # Create the OpenCV display window for the RGB image
        self.cv_window_name = self.node_name
        
        
        
            
        # Initialize the Region of Interest and its publisher
        self.ROI = RegionOfInterest()
        self.roi_pub = rospy.Publisher("/roi", RegionOfInterest, queue_size=1)    

        # Create the cv_bridge object
        self.bridge = CvBridge()
        
        # Subscribe to the camera image and depth topics and set
        # the appropriate callbacks
        #self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback, queue_size=1)
        
        rospy.loginfo("Waiting for image topics...")
        rospy.wait_for_message("input_rgb_image", Image)
        rospy.loginfo("Ready.")

    def image_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Convert the image to a numpy array since most cv2 functions
        # require numpy arrays.
        frame = np.array(frame, dtype=np.uint8)
        
        # Process the frame using the process_image() function
        (crop_lines, x_offset, width) = self.process_image(frame)

        ROI = RegionOfInterest()
        ROI.x_offset = x_offset
        ROI.width = width
        
        self.roi_pub.publish(ROI)
              
        # Display the image and the detected lines.
        
        cv2.imshow(self.node_name, cv2.addWeighted(frame, 1, crop_lines, 1, 0.0))
        # Process any keyboard commands
        self.keystroke = cv2.waitKey(5)
        if self.keystroke != -1:
            cc = chr(self.keystroke & 255).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")

    
   
    def process_image(self, frame):
        '''Inputs an image and outputs the lines'''

        

        ### Grayscale Transform ###
        image_edit = grayscale_transform(frame)
      
    

        ### Skeletonization ###
        skeleton = skeletonize(image_edit)
        
    

        ### Hough Transform ###
        (crop_lines, crop_lines_hough, x_offset, width) = crop_point_hough(skeleton)

        

        return (crop_lines, x_offset, width)


    
    
    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()  


def grayscale_transform(image_in):
    '''Converts RGB to Grayscale and enhances green values'''
    b, g, r = cv2.split(image_in)
    return 2*g - r - b

def skeletonize(image_in):
    '''Inputs and grayscale image and outputs a binary skeleton image'''
    size = np.size(image_in)
    skel = np.zeros(image_in.shape, np.uint8)

    ret, image_edit = cv2.threshold(image_in, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
    done = False

    while not done:
        eroded = cv2.erode(image_edit, element)
        temp = cv2.dilate(eroded, element)
        temp = cv2.subtract(image_edit, temp)
        skel = cv2.bitwise_or(skel, temp)
        image_edit = eroded.copy()

        zeros = size - cv2.countNonZero(image_edit)
        if zeros == size:
            done = True

    return skel


def crop_point_hough(crop_points):
    '''Iterates though Hough thresholds until optimal value found for
       the desired number of crop rows. Also does filtering.
    '''

    height = len(crop_points)
    width = len(crop_points[0])
    
    hough_thresh = HOUGH_THRESH_MAX
    rows_found = False

    while hough_thresh > HOUGH_THRESH_MIN and not rows_found:
        crop_line_data = cv2.HoughLines(crop_points, HOUGH_RHO, HOUGH_ANGLE, hough_thresh)

        crop_lines = np.zeros((height, width, 3), dtype=np.uint8)
        crop_lines_hough = np.zeros((height, width, 3), dtype=np.uint8)

        if crop_line_data != None:

            # get rid of duplicate lines. May become redundant if a similarity threshold is done
            crop_line_data_1 = tuple_list_round(crop_line_data[:, 0, :], -1, 4)
            crop_line_data_2 = []
            x_offsets = []

            crop_lines_hough = np.zeros((height, width, 3), dtype=np.uint8)
            for (rho, theta) in crop_line_data_1:

                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a*rho
                y0 = b*rho
                point1 = (int(round(x0+1000*(-b))), int(round(y0+1000*(a))))
                point2 = (int(round(x0-1000*(-b))), int(round(y0-1000*(a))))
                cv2.line(crop_lines_hough, point1, point2, (0, 0, 255), 2)


            for curr_index in range(len(crop_line_data_1)):
                (rho, theta) = crop_line_data_1[curr_index]

                is_faulty = False
                if ((theta >= ANGLE_THRESH) and (theta <= math.pi-ANGLE_THRESH)) or (theta <= 0.0001):
                    is_faulty = True

                else:
                    for (other_rho, other_theta) in crop_line_data_1[curr_index+1:]:
                        if abs(theta - other_theta) < THETA_SIM_THRESH:
                            is_faulty = True
                        elif abs(rho - other_rho) < RHO_SIM_THRESH:
                            is_faulty = True

                if not is_faulty:
                    crop_line_data_2.append( (rho, theta) )



            for (rho, theta) in crop_line_data_2:

                a = math.cos(theta)
                b = math.sin(theta)
                c = math.tan(theta)
                x0 = a*rho
                y0 = b*rho
                point1 = (int(round(x0+1000*(-b))), int(round(y0+1000*(a))))
                point2 = (int(round(x0-1000*(-b))), int(round(y0-1000*(a))))
                #cv2.circle(crop_lines, (np.clip(int(round(a*rho+c*(0.5*height))),0 ,239), 0), 4, (255,0,0), -1)
                #cv2.circle(crop_lines, (np.clip(int(round(a*rho-c*(0.5*height))),0 ,239), height), 4, (255,0,0), -1)
                cv2.circle(crop_lines, (np.clip(int(round(rho/a)),0 ,239), 0), 5, (255,0,0), -1)
                #cv2.circle(img,(447,63), 63, (0,0,255), -1)
                x_offsets.append(np.clip(int(round(rho/a)),0 ,239))
                cv2.line(crop_lines, point1, point2, (0, 0, 255), 2)


            if len(crop_line_data_2) >= NUMBER_OF_ROWS:
                rows_found = True


        hough_thresh -= HOUGH_THRESH_INCR

    if rows_found == False:
        print(NUMBER_OF_ROWS, "rows_not_found")

    x_offset = min (x_offsets)
    width = max (x_offsets) - min (x_offsets)


    return (crop_lines, crop_lines_hough, x_offset, width)


def tuple_list_round(tuple_list, ndigits_1=0, ndigits_2=0):
    '''Rounds each value in a list of tuples to the number of digits
       specified
    '''
    new_list = []
    for (value_1, value_2) in tuple_list:
        new_list.append( (round(value_1, ndigits_1), round(value_2, ndigits_2)) )

    return new_list
    
def main(args):       
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv2.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    
