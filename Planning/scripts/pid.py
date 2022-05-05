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
import matplotlib.pyplot as plt
from tf.transformations import quaternion_from_euler
class PID():
    def __init__(self,target_pose,mode):
        print("Running PID")
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.controller_status_publisher = rospy.Publisher('/Controller_Status', String, queue_size=1)
        self.diff_error_publisher =  rospy.Publisher('/error_diff', Float64, queue_size=1)
        self.integral_error_publisher =  rospy.Publisher('/error_integ', Float64, queue_size=1)
        self.current_error_publisher =  rospy.Publisher('/error_current', Float64, queue_size=1)
        self.position = Point()
        self.move_cmd = Twist()
        self.r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        self.target_pose = target_pose
        self.kp_distance = 0.8
        self.ki_distance = 0.03
        self.kd_distance = 0.5
        self.kp_angle = 0.75
        self.ki_angle = 0.02
        self.kd_angle = 0.2
        self.counter = 0

    def do_controller_stuff(self, goal_x, goal_y, goal_z):
        (position, rotation) = self.get_odom()
        last_rotation = 0
        linear_speed = 1    #kp_distance
        angular_speed = 1  #kp_angular

        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        #distance is the error for length, x,y
        distance = goal_distance
        previous_distance = 0
        total_distance = 0
        previous_angle = 0
        total_angle = 0
        print("Goals are: ", (goal_x,goal_y,goal_z))
        print("Current POSE: ",(position,rotation))
        # print("-------------Moving to Point!----------")
        total_angle_error = 0.0
        previous_angle_error = 0.0
        while distance > 0.04:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            sin = goal_y - y_start
            cos = goal_x- x_start
            path_angle = atan2(sin,cos)
            flag = False
            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    print("==============Conditon 1")
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    print("==============Conditon 2")
                    path_angle = 2*pi + path_angle
                    flag = True
            if last_rotation > pi-0.1 and rotation <= 0:
                print("==============Conditon 3")
                rotation = 2*pi + rotation
                flag = True
            elif last_rotation < -pi+0.1 and rotation > 0:
                print("==============Conditon 4")
                rotation = -2*pi + rotation
            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            diff_angle = path_angle - previous_angle
            diff_distance = distance - previous_distance
            if round(goal_z-pi)==0:
                if rotation<0:
                    rotation+=2*pi
            current_angle_error = goal_z-rotation
            diff_angle_error = current_angle_error - previous_angle_error
            integral_angle_error = abs(current_angle_error)+total_angle_error
            control_signal_distance = self.kp_distance*distance + self.ki_distance*total_distance + self.kd_distance*diff_distance
            control_signal_angle = self.kp_angle*current_angle_error + self.ki_angle*integral_angle_error + self.kd_angle*diff_angle_error
            if flag:
                print("Flagged angle:",control_signal_angle)

            self.move_cmd.angular.z = control_signal_angle    

            self.move_cmd.linear.x = min(control_signal_distance, 0.1)
            if self.move_cmd.angular.z > 0:
                self.move_cmd.angular.z = min(self.move_cmd.angular.z, 1.5)
            else:
                self.move_cmd.angular.z = max(self.move_cmd.angular.z, -1.5)
            rospy.logdebug("z angular:", self.move_cmd.angular.z)
            rospy.logdebug("Goal rotation: ",goal_z)
            rospy.logdebug("current rotation:",rotation)
            rospy.logdebug("Current angle error:",current_angle_error)
            rospy.logdebug("previous angle_error:",previous_angle_error)
            rospy.logdebug("integral_error:",total_angle_error)
            rospy.logdebug("diff_error:",diff_angle_error)
            rospy.logdebug("Angle Control signal:",self.move_cmd.angular.z)
            rospy.logdebug("-------")
            self.diff_error_publisher.publish(diff_angle_error)
            self.integral_error_publisher.publish(total_angle_error)
            self.current_error_publisher.publish(current_angle_error)
            if current_angle_error>pi:
                raise Exception
            last_rotation = rotation
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
            previous_distance = distance
            previous_angle = path_angle
            previous_angle_error = current_angle_error
            total_angle_error+=current_angle_error
            total_distance = total_distance + distance
            total_angle+=path_angle
        (position, rotation) = self.get_odom()
        rospy.logdebug("Current position and rotation are: ", (position, rotation))
        rospy.logdebug("--------Rotating in place!----------")
        while abs(rotation - goal_z) > 0.05:
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = 0.4
                else:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = -0.4
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = -0.4
                else:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = 0.4
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
        final_pos,final_rot = self.get_odom()
        print("reached!: ",(final_pos,final_rot))
        self.cmd_vel.publish(Twist())

    def publish_velocity(self):
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
        self.counter += 1
        (goal_x, goal_y, goal_z) = self.target_pose
        rounded_x = round(position.x)
        rounded_y = round(position.y)
        print("***** goal:", rounded_x, rounded_y, goal_x, goal_y, goal_z)
        METER_SIZE = 1.0
        NUM_CHECKPOINTS = 1
        ROUND_DIGITS = 2
        step_x = METER_SIZE / NUM_CHECKPOINTS
        step_y = METER_SIZE / NUM_CHECKPOINTS
        if goal_x == rounded_x:
            step_x = 0
        if goal_y == rounded_y:
            step_y = 0
        for i in range(1, NUM_CHECKPOINTS + 1):
            if True or goal_x > rounded_x or goal_y > rounded_y:
                new_goal_x = abs(round(goal_x - step_x * (NUM_CHECKPOINTS - i), ROUND_DIGITS))
                new_goal_y = abs(round(goal_y - step_y * (NUM_CHECKPOINTS - i), ROUND_DIGITS))
            else: # Never reaches here
                new_goal_x = abs(round(rounded_x - step_x * (i), ROUND_DIGITS))
                new_goal_y = abs(round(rounded_y - step_y * (i), ROUND_DIGITS))
            print("***** checkpoint:", i, goal_x, goal_y, new_goal_x, new_goal_y)
            self.do_controller_stuff(new_goal_x, new_goal_y, goal_z)

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

    def round(self, num):
        return round(num*2)/2.0