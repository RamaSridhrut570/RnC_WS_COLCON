#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
*****************************************************************************************
*
*        		===============================================
*           		    Logistic coBot (LB) Theme (eYRC 2024-25)
*        		===============================================
*
*  This script should be used to implement Task 1B of Logistic coBot (LB) Theme (eYRC 2024-25).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ LB#3716 ]
# Author List:	    [Pisupati Rama Sridhrut, Yamini B. Prabha, Ojasvi Tomar, Anisha Deb]
# Filename:         task1b.py
# Functions:        [ Comma separated list of functions in this file ]
# Nodes:	    Add your publishing and subscribing node
# Example:          Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]
################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros 
import tf_transformations
import tf2_geometry_msgs
from rclpy.time import Time
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image


##################### FUNCTION DEFINITIONS #######################


ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}


def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''

    ############ Function VARIABLES ############

    # You can remove these variables after reading the instructions. These are just for sample.

    area = None
    width = None

    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP : 
    #	->  Recevice coordiantes from 'detectMarkers' using cv2.aruco library 
    #       and use these coordinates to calculate area and width of aruco detected.
    #	->  Extract values from input set of 4 (x,y) coordinates 
    #       and formulate width and height of aruco detected to return 'area' and 'width'.

    ############################################
    
    [[[x1,y1],[x2,y2],[x3,y3],[x4,y4]]] = coordinates #extracting values from coordinates argument passed form main function
    # Use the Shoelace formula to calculate the area
    area = 0.5 * abs((x1*y2 + x2*y3 + x3*y4 + x4*y1) - (y1*x2 + y2*x3 + y3*x4 + y4*x1))

    #calculating the width of the detected marker
    # Calculate the distance between two opposite corners
    diagonal1 = math.sqrt((x1 - x3)**2 + (y1 - y3)**2)
    diagonal2 = math.sqrt((x2 - x4)**2 + (y2 - y4)**2)
    
    # Check if both diagonals are approximately equal (within a small tolerance)
    tolerance = 1e-6
    if abs(diagonal1 - diagonal2) < tolerance:
        width = diagonal1  # Use either diagonal to determine width
    

    '''if len(coordinates) != 4: 
           #raise ValueError("Input should contain exactly 4 sets of coordinates")

    # Calculate width as the Euclidean distance between two opposite corners
    width = ((coordinates[0][0] - coordinates[2][0])*2 + (coordinates[0][1] - coordinates[2][1])*2)*0.5

    # Calculate height as the Euclidean distance between the other two opposite corners
    height = ((coordinates[1][0] - coordinates[3][0])*2 + (coordinates[1][1] - coordinates[3][1])*2)*0.5

    # Calculate the area as the product of width and height
    area = width * height'''
       


    return area, width
    



def aruco_display(corners, ids, rejected, image):
    
	if len(corners) > 0:
		
		ids = ids.flatten()
		
		for (markerCorner, markerID) in zip(corners, ids):
			
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
			
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))

			cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
			cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
			cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
			cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
			
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			#cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
			
			cv2.putText(image, "id="+str(markerID),(cX+5, cY- 15), cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (250, 0, 0), 2)
            #cv2.putText(image, "center",(cX, cY- 25), cv2.FONT_HERSHEY_SIMPLEX,
                #0.5, (0, 0, 0), 2)
			#print("[Inference] ArUco marker ID: {}".format(markerID))
            #opLeft[0], topLeft[1] - 10
			
	return image


def detect_aruco(image):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''

    ############ Function VARIABLES ############
    if image is None:
        return [], [], [], [], []
    # ->  You can remove these variables if needed. These are just for suggestions to let you get started

    # Use this variable as a threshold value to detect aruco markers of certain size.
    # Ex: avoid markers/boxes placed far away from arm's reach position  
    aruco_area_threshold = 1500

    # The camera matrix is defined as per camera info loaded from the plugin used. 
    # You may get this from /camer_info topic when camera is spawned in gazebo.
    # Make sure you verify this matrix once if there are calibration issues.
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

    # The distortion matrix is currently set to 0. 
    # We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

    # We are using 150x150 aruco marker size
    size_of_aruco_m = 0.15

    # You can remove these variables after reading the instructions. These are just for sample.
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
 
    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP : 

    #	->  Convert input BGR image to GRAYSCALE for aruco detection
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cv2.imshow("gray_image", gray_image)
    cv2.waitKey(100)
    #   ->  Use these aruco parameters-
    #   ->  Dictionary: 4x4_50 (4x4 only until 50 aruco IDs)

    aruco_type = "DICT_4X4_50"

    #aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    cv2.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
    aruco_params = cv2.aruco.DetectorParameters_create()
   

    #   ->  Detect aruco marker in the image and store 'corners' and 'ids'
    #   ->  HINT: Handle cases for empty markers detection. 
    #corners, ids, _ = cv2.aruco.detectMarkers(gray_image, aruco_dict, parameters=aruco_params)
    #corners, ids, rejected= cv2.aruco.detectMarkers(gray_image, aruco_dict, parameters=aruco_params)
    (corners, ids, rejected) = cv2.aruco.detectMarkers(gray_image, cv2.aruco_dict,parameters=aruco_params)
    
    
    #image=aruco_display(corners, ids, rejected, gray_image)
    #   ->  Draw detected marker on the image frame which will be shown later

    #   ->  Loop over each marker ID detected in frame and calculate area using function defined above (calculate_rectangle_area(coordinates))
    #print('in detect aruco')
    if ids is not None and len(ids) > 0:
        #print('In if condition ofdetect aruco')
        for i, marker_id in enumerate(ids):
            #print('Infor loop ofdetect aruco')
           
            #cv2.aruco.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, 0.1)
            
            # Calculate the area of the detected marker
            if corners is not None:
                 #print(corners[i])
                 area = calculate_rectangle_area(corners[i])
                 #print("areaa is")
                 #print(area)
                 
            else:
                 print("corners none")

            #   ->  Remove tags which are far away from arm's reach positon based on some threshold defined
            #print(area)
            if area[0] > aruco_area_threshold:
                # Draw the detected marker on the image
                cv2.aruco.drawDetectedMarkers(image, corners)
                image_1=aruco_display(corners, ids, rejected, image) 
                #cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)   

                #   ->  Calculate center points aruco list using math and distance from RGB camera using pose estimation of aruco marker
                #   ->  HINT: You may use numpy for center points and 'estimatePoseSingleMarkers' from cv2 aruco library for pose estimation
                # Estimate the pose of the detected marker
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], size_of_aruco_m, cam_mat, dist_mat)

                # Calculate the center point of the marker
                center = np.mean(corners[i][0], axis=0)
                #cv2.circle(image, (center), 4, (0, 0, 255), -1)
                # Calculate the distance from the RGB camera (assuming Z-axis is depth)
               
                distance = tvec[0][0][1]
                #distance = np.linalg.norm(tvec)
                #print('no error in dis')

                # Calculate the angle (you may need to adjust this based on your requirements)
              
                angle = rvec[0][0][2]
                #angle= np.degrees(np.arctan2(rvec[0][0][1], rvec[0][0][2]))


                # Calculate the width of the marker (you may need to adjust this based on your requirements)
                #width = np.linalg.norm(corners[i][0][0] - corners[i][0][1])
                width = np.abs(corners[i][0][0][0] - corners[i][0][1][0])

                
                # Draw frame axes
              
                cv2.drawFrameAxes(image_1, cam_mat, dist_mat, rvec, tvec,1)
                x=center[0]
                y=center[1]
                c_x=round(x)
                c_y=round(y)
               
                cv2.circle(image_1, (c_x,c_y), 5, (0, 0, 255), -1)

              
                cv2.imshow('Detected Markers', image_1) 
                cv2.waitKey(100)


                print("data for arucoId: ",ids[i])
                print("x coordinate of centre: ",c_x)
                print("y coordinate of centre: ",c_y)
                print("angle is:  ",angle)
               # Append the details to the respective lists
                center_aruco_list.append(center)
                distance_from_rgb_list.append(distance)
                angle_aruco_list.append(angle)
                width_aruco_list.append(width)
            
    ############################################
    #print('detect aruco',center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids)

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids


##################### CLASS DEFINITION #######################

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                    # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        
        
        self.cv_image = None                                               # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                       # depth image variable (from depthimagecb())


    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type
        try:   
             cv2_depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
             cv2.imshow("Depth Image", cv2_depth_image)
             cv2.waitKey(100)

            #   ->  HINT: You may use CvBridge to do the same
            ############################################
        except Exception as e:
             print(f"Error converting depth image: {e}") 

        self.depth_image=cv2_depth_image
        

        if self.depth_image is None:
                     print('no depth in construct')
        return cv2_depth_image  



    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Color Image", cv_image)
            cv2.waitKey(100)  # Convert the ROS Image message to a CV2 image

            #   ->  HINT:   You may use CvBridge to do the same
            #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
            #               You may use cv2 functions such as 'flip' and 'rotate' to do the same
        except Exception as e:
            print("Error converting ROS Image to CV2: %s", str(e))   #will show error you have to tun in python environment.

        self.cv_image=cv_image
        #self.get_logger().info("Received an image")

        return cv_image  # Handle the error as needed

       
        

        ############################################

    
    def process_image(self):
        '''
        Description: Timer function used to detect ArUco markers and publish tf on estimated poses.
        '''
        # Camera parameters (intrinsics)
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
        
        trans = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
        c_quat = trans.transform.rotation
        cam_quaternion = [c_quat.x, c_quat.y, c_quat.z, c_quat.w]
        c_roll, c_pitch, c_yaw = tf_transformations.euler_from_quaternion(cam_quaternion)
        
        print(f"Camera Orientation: Roll: {math.degrees(c_roll)}, Pitch: {math.degrees(c_pitch)}, Yaw: {math.degrees(c_yaw)}")
        # Detect ArUco markers
        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids = detect_aruco(self.cv_image)
        
        if ids is not None:
            for i in range(min(len(angle_aruco_list), 3)):  # Process up to 3 markers

                # Correct the ArUco angle and get quaternion
                angle = angle_aruco_list[i]
                corrected_angle = (0.788 * angle) - ((angle ** 2) / 3160)
                if angle<-0.8:
                    quaternion = tf_transformations.quaternion_from_euler(1.57, c_pitch,1.57-corrected_angle)
                elif angle>0.8:
                    quaternion = tf_transformations.quaternion_from_euler(1.57, -c_pitch,1.57-corrected_angle)
                elif angle<0.5 and angle>-0.5:
                    quaternion = tf_transformations.quaternion_from_euler(1.57-c_pitch, 0,1.57-corrected_angle)
                roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
                
                # Print the Euler angles
                print("Data for id:{i}")
                print(f"Roll: {math.degrees(roll)} degrees, Pitch: {math.degrees(pitch)} degrees, Yaw: {math.degrees(yaw)} degrees,correctedangle:{math.degrees(corrected_angle)}")

                # Depth value in meters
                marker_y = int(center_aruco_list[i][1])
                marker_x = int(center_aruco_list[i][0])
                distance_from_rgb = self.depth_image[marker_y][marker_x] / 1000

                # Convert pixel coordinates to metric (camera frame) coordinates
                x = distance_from_rgb * (sizeCamX - center_aruco_list[i][0] - centerCamX) / focalX
                y = distance_from_rgb * (sizeCamY - center_aruco_list[i][1] - centerCamY) / focalY
                z = distance_from_rgb
                
                angle_in_degrees = 90
                rad = np.deg2rad(angle_in_degrees)

                # Rotate the coordinates around the Y-axis (for 90-degree rotation)
                rotationMatrix_1 = np.array([[np.cos(rad), -np.sin(rad)], [np.sin(rad), np.cos(rad)]])
                points_xz = np.array([x, z])
                x, z = np.dot(points_xz, rotationMatrix_1)
                
                # Rotate the coordinates around the X-axis (to maintain vertical alignment)
                rotationMatrix_2 = np.array([[np.cos(-rad), -np.sin(-rad)], [np.sin(-rad), np.cos(-rad)]])
                points_yz = np.array([y, z])
                y, z = np.dot(points_yz, rotationMatrix_2)

                # Create and publish transform for the ArUco marker (camera_link -> cam_<marker_id>)
                tf_msg = TransformStamped()
                tf_msg.header.stamp = self.get_clock().now().to_msg()
                tf_msg.header.frame_id = 'camera_link'
                tf_msg.child_frame_id = 'cam_' + str(ids[i])
                
                tf_msg.transform.translation.x = x
                tf_msg.transform.translation.y = y
                tf_msg.transform.translation.z = z
                
                tf_msg.transform.rotation.x = quaternion[0]
                tf_msg.transform.rotation.y = quaternion[1]
                tf_msg.transform.rotation.z = quaternion[2]
                tf_msg.transform.rotation.w = quaternion[3]


                self.br.sendTransform(tf_msg)

                # Additional step: Transform from base_link to marker (if needed)
                try:
                    base_to_obj = self.tf_buffer.lookup_transform('base_link', 'cam_' + str(ids[i]), Time())
                    self.br.sendTransform(base_to_obj)

                    # Publish another transform for object frame
                    obj_tf_msg = TransformStamped()
                    obj_tf_msg.header.stamp = self.get_clock().now().to_msg()
                    obj_tf_msg.header.frame_id = 'base_link'
                    obj_tf_msg.child_frame_id = 'obj_' + str(ids[i][0])
                    obj_tf_msg.transform = base_to_obj.transform

                    self.br.sendTransform(obj_tf_msg)

                except tf2_ros.LookupException as e:
                    self.get_logger().error(f'Error looking up transform: {e}')



##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node
	
    node.get_logger().info('Node created: Aruco tf process')        # logging information
	
    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()
