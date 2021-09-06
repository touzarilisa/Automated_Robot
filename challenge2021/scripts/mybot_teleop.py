#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys, termios, tty
import click
from geometry_msgs.msg import Twist

topic=rospy.get_param('/which_topic')


rospy.init_node('mybot_teleop', anonymous=True)
#velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
velocity_publisher = rospy.Publisher(topic, Twist, queue_size=10)

move_cmd = Twist()


# Arrow keys codes
keys = {'\x1b[A':'up', '\x1b[B':'down', '\x1b[C':'right', '\x1b[D':'left', 's':'stop', 'q':'quit'}

if rospy.has_param('linear_scale'):
    lin_scale=rospy.get_param('linear_scale')
else:
    lin_scale=1.0

if rospy.has_param('angular_scale'):
    ang_scale=rospy.get_param('angular_scale')
else:
    ang_scale=1.0

if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            rate = rospy.Rate(10)

           # lin_scale=rospy.get_param('/linear_scale')
           # ang_scale=rospy.get_param('/angular_scale')

            # Get character from console
            mykey = click.getchar()
            if mykey in keys.keys():
                char=keys[mykey]

            if char == 'up':    # UP key
                move_cmd.linear.x = lin_scale 
                move_cmd.angular.z=0
                velocity_publisher.publish(move_cmd)


            if char == 'down':  # DOWN key
                
                move_cmd.linear.x =-lin_scale
                move_cmd.angular.z=0
                velocity_publisher.publish(move_cmd)

            if char == 'left':  # RIGHT key
            
                move_cmd.angular.z=-ang_scale
                move_cmd.linear.x=0
                velocity_publisher.publish(move_cmd)

            if char == 'right': # LEFT
               
                move_cmd.linear.x=0
                move_cmd.angular.z =ang_scale 
                velocity_publisher.publish(move_cmd)
	        
            if char =="stop":
                move_cmd.linear.x = 0
                move_cmd.linear.y = 0
                move_cmd.linear.z = 0
                move_cmd.angular.x = 0
                move_cmd.angular.y = 0
                move_cmd.angular.z = 0
                velocity_publisher.publish(move_cmd)

            if char == "quit":  # QUIT
            
                move_cmd.linear.x = 0    
                move_cmd.linear.y = 0
                move_cmd.linear.z = 0
                move_cmd.angular.x = 0
                move_cmd.angular.y = 0
                move_cmd.angular.z = 0
                velocity_publisher.publish(move_cmd)
                break
        except rospy.ROSInterruptException:
            pass

