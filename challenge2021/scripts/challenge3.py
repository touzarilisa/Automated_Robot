#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import sys, termios, tty
import click
import cv_bridge 
import cv2

from sensor_msgs.msg import LaserScan  # type de msg publié par gazebo
from geometry_msgs.msg import Twist  # type de msg à publier sur cmd_vel
from sensor_msgs.msg import Image 

move_cmd = Twist()  # msg à publier sur cmd_vel
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
 
global detecte
detecte=0

global count
count=0        	
def read_pose_callback(obstacle_pose):
    
    regions = {
        'fright':  min(obstacle_pose.ranges[315:349]), # 300 339 
        'front':  min(min(obstacle_pose.ranges[0:9],obstacle_pose.ranges[350:359])),         
        'fleft':   min(obstacle_pose.ranges[10:45]), # 10 49
    }   
    if detecte==0:	
        take_action(regions)
    
def take_action(regions):
	
	safe_dist=0.3  #0.35
	pas_x=0.1 #0.1
	pas_z=0.3  #1
	#move_cmd = Twist()
	linear_x = 0
	angular_z = 0

	if regions['front']>safe_dist:     	  	
		if (regions['fright']>regions['fleft'] ):
			linear_x = pas_x
			angular_z = - pas_z
		
		elif regions['fright']<regions['fleft']:
			linear_x = pas_x
			angular_z = pas_z
			
		else :
			linear_x=pas_x
		
	elif	regions['front']<safe_dist:
		if regions['fright']>regions['fleft'] :	
			angular_z = -pas_z

		elif regions['fright']<regions['fleft']:
			angular_z = pas_z
			
	move_cmd.linear.x = linear_x
	move_cmd.angular.z = angular_z
	velocity_publisher.publish(move_cmd) 	
	
def read_image_callback(image_msg):
	global detecte
	global count
	
	# traitement à faire sur image_msg!!
	cvBridge = cv_bridge.CvBridge()
	
	# transform the image to open cv format
	cvImage = cvBridge.imgmsg_to_cv2( image_msg , desired_encoding='bgr8')
	
	# Change color represtation from BGR to HSV
	hsv = cv2.cvtColor (cvImage , cv2.COLOR_BGR2HSV)
	
	# Image binarisation
	lower_yellow = np.array([20,100,100])
	upper_yellow = np.array([50,255,255])			
	
	lower_blue = np.array([90,50,50])
	upper_blue = np.array([120,255,255])
	
	mask_yellow= cv2.inRange(hsv, lower_yellow, upper_yellow)  
	mask_blue= cv2.inRange(hsv, lower_blue, upper_blue) 

	h, w, d = cvImage.shape
	search_top = 3*h/4
	search_bot = 3*h/4 + 20
		
	mask_yellow[0:int(search_top), 0:w] = 0
	mask_yellow[int(search_bot):h, 0:w] = 0  
	
	mask_blue[0:int(search_top), 0:w] = 0
	mask_blue[int(search_bot):h, 0:w] = 0        

	# Compute the mask moments
	M_y = cv2.moments(mask_yellow)
	M_b=cv2.moments(mask_blue)
	
	
	if M_y["m00"]!=0:		
		detecte=1								
		# Calculate x , y coordinate of center
		cX = int (M_y["m10"]/M_y["m00"])
		cY = int (M_y["m01"]/M_y["m00"])
		cv2.circle(cvImage, (cX, cY), 20, (0,0,255), -1)
		
		err = cX - w/2	
		if detecte==1:
			if count<23:
				move_cmd.linear.x = 0.15
				move_cmd.angular.z = -float(err) / 100 		
				velocity_publisher.publish(move_cmd)
				count=count+1
			else :
				move_cmd.linear.x = 0.0
				move_cmd.angular.z = 0.0		
				velocity_publisher.publish(move_cmd)
	elif M_b["m00"]!=0:
		detecte=1								
		# Calculate x , y coordinate of center
		cX = int (M_b["m10"]/M_b["m00"])
		cY = int (M_b["m01"]/M_b["m00"])
		cv2.circle(cvImage, (cX, cY), 20, (0,0,255), -1)
		
		err = cX - w/2	
		if detecte==1:
			
			move_cmd.linear.x = 0.15
			move_cmd.angular.z = -float(err) / 100 		
			velocity_publisher.publish(move_cmd)
						
	else:
		detecte=0
		
	cv2.imshow("window", cvImage)		
	cv2.waitKey(3)
	
if __name__ == '__main__':
    try:
        rospy.init_node('challenge31', anonymous=True)
        rospy.Subscriber("/scan",LaserScan,read_pose_callback) 
        rospy.Subscriber("/camera/image",Image,read_image_callback)        
        # And then ... wait for the node to be terminated
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

