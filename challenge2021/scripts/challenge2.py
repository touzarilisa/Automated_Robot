#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan  # type de msg publié par gazebo
from geometry_msgs.msg import Twist  # type de msg à publier sur cmd_vel
from nav_msgs.msg import Odometry # type de message pour la pose du robot

move_cmd = Twist()  # msg à publier sur cmd_vel
coordonne= Odometry() #pose du robot
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

global coord_y
coord_y=0

def read_odom_callback(odom):
    global coord_y
    coordonne=odom
    coord_y=coordonne.pose.pose.position.y
        	
def read_pose_callback(obstacle_pose):
    global coord_y
    regions = {
        'fright':  min(obstacle_pose.ranges[270:289]), 
        'front':  min(min(obstacle_pose.ranges[0:9],obstacle_pose.ranges[350:359])),        
        'fleft':   min(obstacle_pose.ranges[70:89]),  
    }   

    take_action(regions)
    
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
        
	if ( coord_y<=7.5):	
				
		move_cmd.linear.x = linear_x
		move_cmd.angular.z = angular_z
		velocity_publisher.publish(move_cmd) 	
   	    	
	else :

		move_cmd.linear.x = 0
		move_cmd.angular.z = 0 
		velocity_publisher.publish(move_cmd)            

if __name__ == '__main__':
    try:
        rospy.init_node('challenge2', anonymous=True)
        rospy.Subscriber("/scan",LaserScan,read_pose_callback) 
        rospy.Subscriber('/odom',Odometry,read_odom_callback)       
        # And then ... wait for the node to be terminated
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

