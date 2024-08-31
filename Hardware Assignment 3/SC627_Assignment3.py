#!/usr/bin/env python3
#! /usr/bin/env python


import rospy
import time
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import math
import matplotlib.pyplot as plt
 
position_1_ = Point()
position_2_ = Point()
position_3_ = Point()
position_4_ = Point()
center_of_mass_ = Point()
yaw_1_ = 0
yaw_2_ = 0
yaw_3_ = 0
yaw_4_ = 0
prec_yaw_ = math.pi/180
sin_prec_ = 0.01
speed_linear_ = 0.1
ang_weight_ = 0.05
sine_weight_ = 0.5
bias_ = 0.8
vel_modulus_ = 0.3
start_time_ = time.time()
time_axis_ = []
first_yaw_ = []
second_yaw_ = []
third_yaw_ = []
fourth_yaw_ = []
first_pos_x_ = []
first_pos_y_ = []
second_pos_x_ = []
second_pos_y_ = []
third_pos_x_ = []
third_pos_y_ = []
fourth_pos_x_ = []
fourth_pos_y_ = []
com_pos_x_ = []
com_pos_y_ = []
should_plot_ = 0 
delay_ = 5e-2 



def normalize_angle(angle):
    if(angle < 0):
        angle = angle + 2 * math.pi
    if(math.fabs(angle) > 2*math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def odom_callback_1(msg):
    global position_1_, yaw_1_
    position_1_ = msg.pose.pose.position
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_1_ = euler[2]
    yaw_1_ = normalize_angle(yaw_1_)

def odom_callback_2(msg):
    global position_2_, yaw_2_
    position_2_ = msg.pose.pose.position
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_2_ = euler[2]
    yaw_2_ = normalize_angle(yaw_2_)  

def odom_callback_3(msg):
    global position_3_, yaw_3_
    position_3_ = msg.pose.pose.position
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_3_ = euler[2]
    yaw_3_ = normalize_angle(yaw_3_)

def odom_callback_4(msg):
    global position_4_, yaw_4_
    position_4_ = msg.pose.pose.position
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_4_ = euler[2]
    yaw_4_ = normalize_angle(yaw_4_)

#Test functions to see if linear motion is working fine 
def linear_1():
    global position_1_, speed_linear_
    move_the_bot.linear.x = speed_linear_
    move_the_bot.angular.z = 0
    publish_to_cmd_vel_1.publish(move_the_bot)
    print("Bot 1 is moving", position_1_)

def linear_2():
    global position_2_, speed_linear_
    move_the_bot.linear.x = speed_linear_
    move_the_bot.angular.z = 0
    publish_to_cmd_vel_2.publish(move_the_bot)
    print("Bot 2 is moving", position_2_)

def linear_3():
    global position_3_, speed_linear_
    move_the_bot.linear.x = speed_linear_
    move_the_bot.angular.z = 0
    publish_to_cmd_vel_3.publish(move_the_bot)
    print("Bot 3 is moving", position_3_)

def linear_4():
    global position_4_, speed_linear_
    move_the_bot.linear.x = speed_linear_
    move_the_bot.angular.z = 0
    publish_to_cmd_vel_4.publish(move_the_bot)
    print("Bot 4 is moving", position_4_)

#Test functions to see if all the angular motions are working fine 
def angular_1(des_yaw_1):
    global yaw_1_, prec_yaw_
    err_yaw = normalize_angle(des_yaw_1 - yaw_1_)
    twist_msg = Twist()
    twist_msg = Twist()
    if math.fabs(err_yaw) > prec_yaw_:
        twist_msg.angular.z = 0.1 if err_yaw > 0 else -0.1
    publish_to_cmd_vel_1.publish(twist_msg)

def angular_2(des_yaw_2):
    global yaw_2_, prec_yaw_
    err_yaw = normalize_angle(des_yaw_2 - yaw_1_)
    twist_msg = Twist()
    twist_msg = Twist()
    if math.fabs(err_yaw) > prec_yaw_:
        twist_msg.angular.z = 0.1 if err_yaw > 0 else -0.1
    publish_to_cmd_vel_2.publish(twist_msg)    

def angular_3(des_yaw_3):
    global yaw_3_, prec_yaw_
    err_yaw = normalize_angle(des_yaw_3 - yaw_3_)
    twist_msg = Twist()
    twist_msg = Twist()
    if math.fabs(err_yaw) > prec_yaw_:
        twist_msg.angular.z = 0.1 if err_yaw > 0 else -0.1
    publish_to_cmd_vel_3.publish(twist_msg)

def angular_4(des_yaw_4):
    global yaw_4_, prec_yaw_
    err_yaw = normalize_angle(des_yaw_4 - yaw_4_)
    twist_msg = Twist()
    twist_msg = Twist()
    if math.fabs(err_yaw) > prec_yaw_:
        twist_msg.angular.z = 0.1 if err_yaw > 0 else -0.1
    publish_to_cmd_vel_4.publish(twist_msg)

#Consensus using magnitudes of yaws directly
def consensus_1():
    global yaw_1_, yaw_2_, yaw_3_, yaw_4_, ang_weight_, prec_yaw_
    err_yaw = yaw_2_ + yaw_3_ + yaw_4_ - 3*yaw_1_
    twist_msg = Twist()
    if math.fabs(err_yaw) > prec_yaw_:
        twist_msg.angular.z = ang_weight_ * err_yaw 
    publish_to_cmd_vel_1.publish(twist_msg) 

def consensus_2():
    global yaw_1_, yaw_2_, yaw_3_, yaw_4_, ang_weight_, prec_yaw_
    err_yaw = yaw_1_ + yaw_3_ + yaw_4_ - 3*yaw_2_
    twist_msg = Twist()
    if math.fabs(err_yaw) > prec_yaw_:
        twist_msg.angular.z = ang_weight_ * err_yaw 
    publish_to_cmd_vel_2.publish(twist_msg)

def consensus_3():
    global yaw_1_, yaw_2_, yaw_3_, yaw_4_, ang_weight_, prec_yaw_
    err_yaw = yaw_2_ + yaw_1_ + yaw_4_ - 3*yaw_3_
    twist_msg = Twist()
    if math.fabs(err_yaw) > prec_yaw_:
        twist_msg.angular.z = ang_weight_ * err_yaw 
    publish_to_cmd_vel_3.publish(twist_msg)

def consensus_4():
    global yaw_1_, yaw_2_, yaw_3_, yaw_4_, ang_weight_, prec_yaw_
    err_yaw = yaw_2_ + yaw_3_ + yaw_1_ - 3*yaw_4_
    twist_msg = Twist()
    if math.fabs(err_yaw) > prec_yaw_:
        twist_msg.angular.z = ang_weight_ * err_yaw 
    publish_to_cmd_vel_4.publish(twist_msg)

#Synchronization using sinusoidal gradient control law
def gradient_alignment_1():
    global yaw_1_, yaw_2_, yaw_3_, yaw_4_, ang_weight_, sin_prec_, vel_modulus_
    sin_err_yaw = math.sin(yaw_1_ - yaw_2_) + math.sin(yaw_1_ - yaw_3_) + math.sin(yaw_1_ - yaw_4_)
    twist_msg = Twist()
    if math.fabs(sin_err_yaw) > sin_prec_:
        twist_msg.angular.z = -sin_err_yaw * sine_weight_
        twist_msg.linear.x = vel_modulus_
    publish_to_cmd_vel_1.publish(twist_msg)

def gradient_alignment_2():
    global yaw_1_, yaw_2_, yaw_3_, yaw_4_, ang_weight_, sin_prec_, vel_modulus_
    sin_err_yaw = math.sin(yaw_2_ - yaw_1_) + math.sin(yaw_2_ - yaw_3_) + math.sin(yaw_2_ - yaw_4_)
    twist_msg = Twist()
    if math.fabs(sin_err_yaw) > sin_prec_:
        twist_msg.angular.z = -sin_err_yaw * sine_weight_
        twist_msg.linear.x = vel_modulus_
    publish_to_cmd_vel_2.publish(twist_msg)


def gradient_alignment_3():
    global yaw_1_, yaw_2_, yaw_3_, yaw_4_, ang_weight_, sin_prec_, vel_modulus_
    sin_err_yaw = math.sin(yaw_3_ - yaw_1_) + math.sin(yaw_3_ - yaw_2_) + math.sin(yaw_3_ - yaw_4_)
    twist_msg = Twist()
    if math.fabs(sin_err_yaw) > sin_prec_:
        twist_msg.angular.z = -sin_err_yaw * sine_weight_
        twist_msg.linear.x = vel_modulus_
    publish_to_cmd_vel_3.publish(twist_msg)


def gradient_alignment_4():
    global yaw_1_, yaw_2_, yaw_3_, yaw_4_, ang_weight_, sin_prec_, vel_modulus_
    sin_err_yaw = math.sin(yaw_4_ - yaw_1_) + math.sin(yaw_4_ - yaw_2_) + math.sin(yaw_4_ - yaw_3_)
    twist_msg = Twist()
    if math.fabs(sin_err_yaw) > sin_prec_:
        twist_msg.angular.z = -sin_err_yaw * sine_weight_
        twist_msg.linear.x = vel_modulus_
    publish_to_cmd_vel_4.publish(twist_msg)


#Balance using sinusoidal gradient control law
def gradient_balance_1():
    global yaw_1_, yaw_2_, yaw_3_, yaw_4_, ang_weight_, sin_prec_, vel_modulus_, bias_
    sin_err_yaw = math.sin(yaw_1_ - yaw_2_) + math.sin(yaw_1_ - yaw_3_) + math.sin(yaw_1_ - yaw_4_)
    twist_msg = Twist()
    if math.fabs(sin_err_yaw) > sin_prec_:
        twist_msg.angular.z = sin_err_yaw * sine_weight_ + bias_ 
        twist_msg.linear.x = vel_modulus_
    publish_to_cmd_vel_1.publish(twist_msg)

def gradient_balance_2():
    global yaw_1_, yaw_2_, yaw_3_, yaw_4_, ang_weight_, sin_prec_, vel_modulus_, bias_
    sin_err_yaw = math.sin(yaw_2_ - yaw_1_) + math.sin(yaw_2_ - yaw_3_) + math.sin(yaw_2_ - yaw_4_)
    twist_msg = Twist()
    if math.fabs(sin_err_yaw) > sin_prec_:
        twist_msg.angular.z = sin_err_yaw * sine_weight_ + bias_ 
        twist_msg.linear.x = vel_modulus_
    publish_to_cmd_vel_2.publish(twist_msg)


def gradient_balance_3():
    global yaw_1_, yaw_2_, yaw_3_, yaw_4_, ang_weight_, sin_prec_, vel_modulus_, bias_
    sin_err_yaw = math.sin(yaw_3_ - yaw_1_) + math.sin(yaw_3_ - yaw_2_) + math.sin(yaw_3_ - yaw_4_)
    twist_msg = Twist()
    if math.fabs(sin_err_yaw) > sin_prec_:
        twist_msg.angular.z = sin_err_yaw * sine_weight_ + bias_ 
        twist_msg.linear.x = vel_modulus_
    publish_to_cmd_vel_3.publish(twist_msg)


def gradient_balance_4():
    global yaw_1_, yaw_2_, yaw_3_, yaw_4_, ang_weight_, sin_prec_, vel_modulus_, bias_
    sin_err_yaw = math.sin(yaw_4_ - yaw_1_) + math.sin(yaw_4_ - yaw_2_) + math.sin(yaw_4_ - yaw_3_)
    twist_msg = Twist()
    if math.fabs(sin_err_yaw) > sin_prec_:
        twist_msg.angular.z = sin_err_yaw * sine_weight_+ bias_ 
        twist_msg.linear.x = vel_modulus_
    publish_to_cmd_vel_4.publish(twist_msg)

#Function to update plot lists, as the name suggests
def update_plots(): 
    global start_time_, time_axis_
    global position_1_, position_2_, position_3_, position_4_, center_of_mass_
    global first_pos_x_, second_pos_x_, third_pos_x_, fourth_pos_x_, com_pos_x_
    global first_pos_y_, second_pos_y_, third_pos_y_, fourth_pos_y_, com_pos_y_
    global yaw_1_, yaw_2_, yaw_3_, yaw_4_
    global first_yaw_, second_yaw_, third_yaw_, fourth_yaw_
    end_time = time.time()
    diff_time = end_time - start_time_
    center_of_mass_.x = 0.25*(position_1_.x + position_2_.x +  position_3_.x + position_4_.x)
    center_of_mass_.y = 0.25*(position_1_.y + position_2_.y +  position_3_.y + position_4_.y)
    center_of_mass_.z = 0.25*(position_1_.z + position_2_.z +  position_3_.z + position_4_.z)
    time_axis_.append(diff_time)
    first_pos_x_.append(position_1_.x)
    first_pos_y_.append(position_1_.y)
    second_pos_x_.append(position_2_.x)
    second_pos_y_.append(position_2_.y)
    third_pos_x_.append(position_3_.x)
    third_pos_y_.append(position_3_.y)
    fourth_pos_x_.append(position_4_.x)
    fourth_pos_y_.append(position_4_.y)
    first_yaw_.append(yaw_1_)
    second_yaw_.append(yaw_2_)
    third_yaw_.append(yaw_3_)
    fourth_yaw_.append(yaw_4_)
    com_pos_x_.append(center_of_mass_.x)
    com_pos_y_.append(center_of_mass_.y)



if __name__ == "__main__":

    rospy.init_node('turtlebot_controller_node')

#Separate topics for each robot have been created 
    rospy.loginfo('The node for robot 1 has been started')
    publish_to_cmd_vel_1 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size = 10)
    subscribe_to_odom_1 = rospy.Subscriber('/tb3_1/odom', Odometry, callback = odom_callback_1)
    rospy.loginfo('The node for robot 2 has been started')
    publish_to_cmd_vel_2 = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size = 10)
    subscribe_to_odom_2 = rospy.Subscriber('/tb3_2/odom', Odometry, callback = odom_callback_2)
    rospy.loginfo('The node for robot 3 has been started')
    publish_to_cmd_vel_3 = rospy.Publisher('/tb3_5/cmd_vel', Twist, queue_size = 10)
    subscribe_to_odom_3 = rospy.Subscriber('/tb3_5/odom', Odometry, callback = odom_callback_3)
    rospy.loginfo('The node for robot 4 has been started')
    publish_to_cmd_vel_4 = rospy.Publisher('/tb3_6/cmd_vel', Twist, queue_size = 10)
    subscribe_to_odom_4 = rospy.Subscriber('/tb3_6/odom', Odometry, callback = odom_callback_4)   
    
    
    move_the_bot = Twist()
    rate_hz = 20
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        #gradient_alignment_1()
        gradient_balance_1()
        time.sleep(delay_)
        #gradient_alignment_2()
        gradient_balance_2()
        time.sleep(delay_)
        #gradient_alignment_3()
        gradient_balance_3()
        time.sleep(delay_)
        #gradient_alignment_4()
        gradient_balance_4()
        time.sleep(delay_)
        update_plots()
        current_time = time.time()
        if current_time - start_time_ >= 40 and should_plot_ == 0:
            plt.figure(1)
            plt.plot(time_axis_, first_yaw_)
            plt.plot(time_axis_, second_yaw_)
            plt.plot(time_axis_, third_yaw_)
            plt.plot(time_axis_, fourth_yaw_)
            plt.figure(2)
            plt.plot(first_pos_x_, first_pos_y_)
            plt.plot(second_pos_x_, second_pos_y_)
            plt.plot(third_pos_x_, third_pos_y_)
            plt.plot(fourth_pos_x_, fourth_pos_y_)
            plt.figure(3)
            plt.plot(time_axis_, com_pos_x_)
            plt.plot(time_axis_, com_pos_y_)
            plt.show()
            should_plot_ = 1
    rospy.spin()

