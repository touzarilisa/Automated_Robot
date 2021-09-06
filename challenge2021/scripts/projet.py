#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import sys, termios, tty
import click
import cv_bridge 
#-import Cvbridge, CvBridgeError
import cv2
import math

from nav_msgs.msg import Odometry # type de message pour la pose du robot
from sensor_msgs.msg import Image  # type de msg publié par gazebo
from sensor_msgs.msg import LaserScan  # type de msg publié par gazebo
from geometry_msgs.msg import Twist  # type de msg à publier sur cmd_vel

move_cmd = Twist()  # msg à publier sur cmd_vel
coordonne= Odometry() #pose du robot
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

global detecte_obstacle
global numero_obstacle
global obstacle
global tourner
global num_challenge
global obstacle_back
global obstacle_right
global detecte
global count

count=0 
detecte_obstacle=False
numero_obstacle=1
tourner=0
num_challenge=1
detecte=0
#global challenge_number

global coord_y
coord_y=0

def read_odom_callback(odom):	
	global coord_y
	global num_challenge
	coordonne=odom
	coord_y=coordonne.pose.pose.position.y
	
	if coord_y<5.7:
		num_challenge=1
	elif coord_y>=5.7 and coord_y<7.5 and num_challenge==1:
		num_challenge=2
		
		move_cmd.linear.x = 0.2
		move_cmd.angular.z = -1		
		velocity_publisher.publish(move_cmd) 
	else :
		num_challenge=3
        	

def read_image_callback(image_msg): # image_msg pour recuperer les images
	global detecte_obstacle
	global numero_obstacle	
	global obstacle
	global tourner
	global num_challenge
	global obstacle_back
	global obstacle_right
	global detecte
	global count
	# traitement à faire sur image_msg!!
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
	
	lower_yellow = np.array([20,100,100])
	upper_yellow = np.array([50,255,255])	
	
	mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
	mask_orange= cv2.inRange(hsv, lower_orange, upper_orange)
	mask_yellow= cv2.inRange(hsv, lower_yellow, upper_yellow)   
	mask=mask_blue+mask_orange
        
        # voir l'explication!!!!
        
	h, w, d = cvImage.shape
	search_top = 3*h/4
	search_bot = 3*h/4 + 20
	        
	mask[0:int(search_top), 0:w] = 0
	mask[int(search_bot):h, 0:w] = 0        
	
	# Compute the mask moments
	M = cv2.moments(mask)
	
	
	mask_orange[0:int(search_top), 0:w] = 0
	mask_orange[int(search_bot):h, 0:w] = 0
	M1=cv2.moments(mask_orange)
	
	mask_blue[0:int(search_top), 0:w] = 0
	mask_blue[int(search_bot):h, 0:w] = 0
	M2=cv2.moments(mask_blue)
	
	mask_yellow[0:int(search_top), 0:w] = 0
	mask_yellow[int(search_bot):h, 0:w] = 0
	M_y = cv2.moments(mask_yellow)
	
	
	
	if tourner==1 and num_challenge==1:
		if obstacle_back!=math.inf and obstacle_back > 0.24: 
			tourner=0
		elif obstacle_back > 0.15 :
			if  obstacle_right==math.inf:
				move_cmd.angular.z= 0.3
				velocity_publisher.publish(move_cmd)	
			elif obstacle_right <=0.07 :
				move_cmd.linear.x = 0.1       #0.15
				move_cmd.angular.z = 0.3	#0.5	
				velocity_publisher.publish(move_cmd)
			elif obstacle_right >=0.07 :
				move_cmd.linear.x = 0.1       #0.15
				move_cmd.angular.z = -0.3	#0.5	
				velocity_publisher.publish(move_cmd)
		
	
		
	elif M["m00"]>0 and num_challenge==1:
			
		if (M1["m00"]==0 and numero_obstacle==1 ):			
			numero_obstacle=2
								
		# Calculate x , y coordinate of center
		cX = int (M["m10"]/M["m00"])
		cY = int (M["m01"]/M["m00"])
		cv2.circle(cvImage, (cX, cY), 20, (0,0,255), -1)
		
		err = cX - w/2	
		if (detecte_obstacle==False ):	
			move_cmd.linear.x = 0.15
			move_cmd.angular.z = -float(err) / 100 		
			velocity_publisher.publish(move_cmd) # publier dans cmd_vel
		elif (detecte_obstacle==True and numero_obstacle==1):
			move_cmd.linear.x = 0.0
			move_cmd.angular.z = 0.0		
			velocity_publisher.publish(move_cmd) # publier dans cmd_vel
			
		elif (detecte_obstacle==True and numero_obstacle==2):
						
			move_cmd.linear.x = 0.0		
			move_cmd.angular.z= 0.0
			velocity_publisher.publish(move_cmd)
			tourner=1
			
	elif M["m00"]<=0 and num_challenge==1:
		
		move_cmd.linear.x = 0
		move_cmd.angular.z = 0		
		velocity_publisher.publish(move_cmd) 
						
						       	        
	cv2.imshow("window", cvImage)	
	cv2.waitKey(3)
	
	if num_challenge==3:
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
		elif M2["m00"]!=0:
			detecte=1								
			# Calculate x , y coordinate of center
			cX = int (M2["m10"]/M2["m00"])
			cY = int (M2["m01"]/M2["m00"])
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


def read_pose_callback(obstacle_pose):
	
	# traitement à faire sur obstacle_pose!!
	global obstacle	
	global detecte_obstacle
	global numero_obstacle
	global num_challenge
	global obstacle_back
	global obstacle_right
	global detecte
	global count
	
	obstacle=0
	obstacle_right=0
	obstacle_back=0
		
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
	
	if (obstacle<=0.27 and numero_obstacle==1):		
		detecte_obstacle=True
	elif (obstacle<=0.25 and numero_obstacle==2):		
		detecte_obstacle=True	
	else :
		detecte_obstacle=False 
			
	if num_challenge==2:		
		regions = {
        	'fright':  min(obstacle_pose.ranges[270:289]),
		'front':  min(min(obstacle_pose.ranges[0:9],obstacle_pose.ranges[340:359])),   
        	'fleft':   min(obstacle_pose.ranges[70:89]),}
		   
		take_action(regions)
		
	if num_challenge==3:
		regions_3 = {
        	'fright':  min(obstacle_pose.ranges[315:349]), # 300 339 
        	'front':  min(min(obstacle_pose.ranges[0:9],obstacle_pose.ranges[350:359])),         
        	'fleft':   min(obstacle_pose.ranges[10:45]), # 10 49
    		}   
		if detecte==0:	
        		take_action_3(regions_3)
    		
def take_action(regions):
	
	global coord_y
	safe_dist=0.35 
	pas_x=0.1 
	pas_z=0.2
	
	linear_x = 0
	angular_z = 0

	if regions['front']>safe_dist:     	  	
		if (regions['fright']>regions['fleft'] ):
			linear_x = pas_x
			angular_z = - pas_z
			
		elif regions['fright']<regions['fleft']:
			linear_x = pas_x
			angular_z = pas_z
			
	elif	regions['front']<safe_dist:
		if regions['fright']>regions['fleft'] :	
			angular_z = -pas_z
					
		elif regions['fright']<regions['fleft']:
			angular_z = pas_z
				
				
	move_cmd.linear.x = linear_x
	move_cmd.angular.z = angular_z
	velocity_publisher.publish(move_cmd) 	
       
def take_action_3(regions):
	
	safe_dist=0.3  
	pas_x=0.1 
	pas_z=0.3 
	
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

              
if __name__ == '__main__':
	try:
		rospy.init_node('projet', anonymous=True)
		rospy.Subscriber("/camera/image",Image,read_image_callback) 
		rospy.Subscriber("/scan",LaserScan,read_pose_callback) 
		rospy.Subscriber('/odom',Odometry,read_odom_callback)         
		# And then ... wait for the node to be terminated
		rospy.spin()

	except rospy.ROSInterruptException:
		pass

