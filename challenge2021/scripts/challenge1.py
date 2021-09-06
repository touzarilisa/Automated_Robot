#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import sys, termios, tty
import click
import cv_bridge 
import cv2
import math


from sensor_msgs.msg import Image  # type de msg publié par gazebo
from sensor_msgs.msg import LaserScan  # type de msg publié par gazebo
from geometry_msgs.msg import Twist  # type de msg à publier sur cmd_vel

move_cmd = Twist()  # msg à publier sur cmd_vel
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

global detecte_obstacle   # detection d'un obstacle
global numero_obstacle    # numero de l'obstacle detecté
global obstacle           # distance par rapport à un obstacle à l'avant du robot
global obstacle_right     # distance par rapport à un obstacle à la droite du robot
global obstacle_back      # distance par rapport à un obstacle à l'arrière du robot
global tourner

detecte_obstacle=False
numero_obstacle=1
tourner=0

def read_image_callback(image_msg): # image_msg pour recuperer les images
	global detecte_obstacle
	global numero_obstacle	
	global obstacle
	global obstacle_right
	global obstacle_back
	global tourner
	# traitement à faire sur image_msg
	cvBridge = cv_bridge.CvBridge()
	
	# transform the image to open cv format
	cvImage = cvBridge.imgmsg_to_cv2( image_msg , desired_encoding='bgr8')
	
	# Change color represtation from BGR to HSV
	hsv = cv2.cvtColor (cvImage , cv2.COLOR_BGR2HSV)
	
	# Image binarisation
	lower_orange = np.array([0,100,100])
	upper_orange = np.array([30,255,255])	
		
	lower_blue = np.array([90,50,50])
	upper_blue = np.array([120,255,255])	
	
	mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
	mask_orange= cv2.inRange(hsv, lower_orange, upper_orange)  
	mask=mask_blue+mask_orange
        
                
	h, w, d = cvImage.shape
	search_top = 3*h/4
	search_bot = 3*h/4 + 20
	
	# Compute the mask moments 
	       
	mask[0:int(search_top), 0:w] = 0
	mask[int(search_bot):h, 0:w] = 0        
	M = cv2.moments(mask)
	
	
	mask_orange[0:int(search_top), 0:w] = 0
	mask_orange[int(search_bot):h, 0:w] = 0
	M1=cv2.moments(mask_orange)
	
	mask_blue[0:int(search_top), 0:w] = 0
	mask_blue[int(search_bot):h, 0:w] = 0
	M2=cv2.moments(mask_blue)
	 
	
	
	if tourner==1:  # dans le cas d'un obstacle fixe
		if obstacle_back!=math.inf and obstacle_back > 0.22: 
			tourner=0
		elif obstacle_back > 0.15 :
			if  obstacle_right==math.inf:
				move_cmd.angular.z= 0.3
				velocity_publisher.publish(move_cmd)	
			elif obstacle_right <=0.07 :
				move_cmd.linear.x = 0.1       
				move_cmd.angular.z = 0.3		
				velocity_publisher.publish(move_cmd)
			elif obstacle_right >=0.07 :
				move_cmd.linear.x = 0.1       
				move_cmd.angular.z = -0.3		
				velocity_publisher.publish(move_cmd)
			
		
	elif M["m00"]>0: 
				
		if (M1["m00"]==0 and numero_obstacle==1 ):			
			numero_obstacle=2
								
		# Calculate x , y coordinate of center
		cX = int (M["m10"]/M["m00"])
		cY = int (M["m01"]/M["m00"])
		cv2.circle(cvImage, (cX, cY), 20, (0,0,255), -1)
		
		err = cX - w/2
			
		if (detecte_obstacle==False):  # dans le cas où il n'y a pas d'obstacle
			
			move_cmd.linear.x = 0.15    
			move_cmd.angular.z = -float(err) / 100   					
			velocity_publisher.publish(move_cmd) # publier dans cmd_vel
						
		elif (detecte_obstacle==True and numero_obstacle==1):  # arret d'urgence
			move_cmd.linear.x = 0.0
			move_cmd.angular.z = 0.0		
			velocity_publisher.publish(move_cmd) # publier dans cmd_vel
			
		elif (detecte_obstacle==True and numero_obstacle==2):  # dans le cas de l'obstacle fixe
						
			move_cmd.linear.x = 0.0		
			move_cmd.angular.z= 0.0
			velocity_publisher.publish(move_cmd)
			tourner=1
			
	elif  M["m00"]<=0:   # fin de ligne
		move_cmd.linear.x = 0.0
		move_cmd.angular.z = 0.0		
		velocity_publisher.publish(move_cmd) 							
		       	        
	cv2.imshow("window", cvImage)	
	cv2.waitKey(3)



def read_pose_callback(obstacle_pose):
	
	global obstacle
	global obstacle_right	
	global obstacle_back
	global detecte_obstacle
	global numero_obstacle
	obstacle=0
	obstacle_right=0
	obstacle_back=0
	# obstacle in front of robot	
	for i in range(0,9):
		obstacle+= obstacle_pose.ranges[i]
	for i in range(350,359):
		obstacle+= obstacle_pose.ranges[i]		
	obstacle =obstacle/20	
	
	
	# obstacle to the right of robot
	for i in range(265,274):
		obstacle_right+= obstacle_pose.ranges[i]
	obstacle_right=obstacle_right/10
	
	
	# obstacle in the back
	for i in range(255,264):
		obstacle_back+= obstacle_pose.ranges[i]
	obstacle_back=obstacle_back/10
	
	
	# detection des obstacles
	if (obstacle<=0.27 and numero_obstacle==1):		
		detecte_obstacle=True
	elif (obstacle<=0.25 and numero_obstacle==2):		
		detecte_obstacle=True			

	else :
		detecte_obstacle=False   
                               
       
              
if __name__ == '__main__':
    try:
        rospy.init_node('challenge1', anonymous=True)
        rospy.Subscriber("/camera/image",Image,read_image_callback) 
        rospy.Subscriber("/scan",LaserScan,read_pose_callback) 
               
        # And then ... wait for the node to be terminated
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

