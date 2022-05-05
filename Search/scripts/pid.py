#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Shashank Rao Marpally", "Rushang Karia", "Kislay Kumar", "Abhyudaya Srinet"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.3"
__maintainers__ = ["Pulkit Verma", "Naman Shah"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import String,Float64
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import subprocess
import os
import time
import math 
from tf.transformations import quaternion_from_euler

class PID():
    def __init__(self,target_pose,mode):
        rospy.logdebug("Running PID")
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.controller_status_publisher = rospy.Publisher('/Controller_Status', String, queue_size=1)

        self.error_x_publisher =  rospy.Publisher('/error_x', Float64, queue_size=1)
        self.error_y_publisher =  rospy.Publisher('/error_y', Float64, queue_size=1)
        self.error_yaw_publisher =  rospy.Publisher('/error_yaw', Float64, queue_size=1)

        self.position = Point()
        self.move_cmd = Twist()
        self.r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        self.target_pose = target_pose
        self.kp_distance = 0.8
        self.ki_distance = 0.03
        self.kd_distance = 0.5
        self.kp_angle = 0.9
        self.ki_angle = 0.05
        self.kd_angle = 0.07

    def publish_error(x,y,yaw):
        return NotImplementedError


    def publish_velocity(self):
        rospy.logdebug("Called publish velocity!")
        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        (position, rotation) = self.get_odom()

        last_rotation = 0
        linear_speed = 1    #kp_distance
        angular_speed = 1  #kp_angular


        (goal_x, goal_y, goal_z) = self.target_pose
 
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        #distance is the error for length, x,y
        distance = goal_distance
        previous_distance = 0
        total_distance = 0

        previous_angle = 0
        total_angle = 0
        rospy.logdebug("Goals are: ", (goal_x,goal_y,goal_z))
        rospy.logdebug("Current POSE: ",(position,rotation))
        rospy.logdebug("-------------Moving to Point!----------")
        while distance > 0.04:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            #path_angle = error
            sin = goal_y - y_start
            cos = goal_x- x_start
            path_angle = atan2(sin,cos)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    rospy.logdebug("Conditon 1")
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    rospy.logdebug("Conditon 2")
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and rotation <= 0:
                rospy.logdebug("Conditon 3")
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rospy.logdebug("Conditon 4")
                rotation = -2*pi + rotation

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            diff_angle = path_angle - previous_angle
            diff_distance = distance - previous_distance

            rospy.logdebug("Current position and rotation  are: ", (position, rotation))
            control_signal_distance = self.kp_distance*distance + self.ki_distance*total_distance + self.kd_distance*diff_distance
            control_signal_angle = self.kp_angle*path_angle + self.ki_angle*total_angle + self.kd_angle*diff_angle

            w = (control_signal_angle) - rotation
            if w>pi:
                self.move_cmd.angular.z = w-2*pi
            else:
                self.move_cmd.angular.z = w
            
            self.move_cmd.linear.x = min(control_signal_distance, 0.1)

            if self.move_cmd.angular.z > 0:
                self.move_cmd.angular.z = min(self.move_cmd.angular.z, 1.5)
            else:
                self.move_cmd.angular.z = max(self.move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
            previous_distance = distance
            total_distance = total_distance + distance

        (position, rotation) = self.get_odom()

        
        rospy.logdebug("--------Rotating in place!----------")
        while abs(rotation - goal_z) > 0.05:
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = 0.5
                else:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = -0.5
                else:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = 0.5
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()

        final_pos,final_rot = self.get_odom()
        rospy.logdebug("reached!: ",(final_pos,final_rot))
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        self.controller_status_publisher.publish(String("Done"))
        return 

    def getkey(self):
        x = x_input
        y = y_input
        z = z_input
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)
            

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
