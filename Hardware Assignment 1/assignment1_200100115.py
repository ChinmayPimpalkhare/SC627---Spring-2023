#!/usr/bin/env python3
#!/usr/bin/env python

#import all necessary things here
import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist 

from tf import transformations
from std_srvs.srv import * 

#Defining all the global variables here
current_position_ = Point()
goal_position_ = Point()
goal_position_.x = -0.4
goal_position_.y = 2
goal_position_.z = 0.0 
yaw_ = 0 
count_ = 0 
epsilon_ = 0
yaw_tol_ = 1
angular_speed_ = 0.2
circum_yaw_tol_ = 5
state_ = 1
left_dist_ = 0
right_dist_ = 0
circ_entry_point_ = Point()
circ_exit_point_ = Point()
circ_entry_goal_dist_ = 0
circ_exit_goal_dist_ = 0
circ_start_point_ = Point()
circ_start_goal_dist_ = 0
slow_down_dist = 0.05
turn_after_slow_dist = 0.1
robot_radius = 0.1
wait_ = False
count_state_time = 0
count_loop = 0 
closest_obstacle_angle_ = 0
closest_obstacle_dist_ = 0 
left_closest_ = 0
right_closest_ = 0
is_collision_along_left_ = True
dist_along_zero_ = 0 
state_desc_ = [
    'Turn to align with the goal',
    'Move straight in absence of any goal and Slow down due to presence of obstacle',
    'Align with the edge of the obstacle'
    'Follow obstacle',
    'Return to closest position along the obstacle path',
    'Goal reached'
]
comments_list = [
    'Comment 1: Check if the if_collision_is_true function is correct or not',
    'Comment 2: Find out if msg.ranges[90] corresponds to left or right'
    'Comment 3: Check whether the robot rotates anticlockwise or clockwise'
]
answers_list = [
    'Answer 1: As per latest changes, the function if_collision_is_true is correct',
    'Answer 2: msg.ranges[90] corresponds to left',
    'Answer 3: Yes, a positive sign does indeed correspond to anticlockwise rotation'
]

def laser_data_callback(msg): 
    global wait_, yaw_, closest_obstacle_angle_, closest_obstacle_dist_, dist_along_zero_
    global left_closest_, right_closest_
    global left_dist_, right_dist_
    dist_along_zero_ = msg.ranges[0]
    left_closest_ = min(msg.ranges[45:135])
    right_closest_ = min(msg.ranges[225:315])
    left_dist_ = msg.ranges[90]
    right_dist_ = msg.ranges[270]
    closest_obstacle_dist_ = min(msg.ranges)
    closest_obstacle_angle_ = msg.ranges.index(closest_obstacle_dist_)
    wait_ = True

def clbk_odom(msg):
    global current_position_, yaw_

    # position
    current_position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]*180//math.pi 
    yaw_ = yaw_ if yaw_ >= 0 else (yaw_ + 360)

def is_collision_along_left():
    global is_collision_along_left_, left_closest_, right_closest_
    if left_closest_ >= right_closest_: 
        is_collision_along_left_ = False
    return is_collision_along_left_

def ret_distance(P1, P2):
    dist = math.sqrt((P1.x - P2.x)**2 + (P1.y - P2.y)**2)
    return dist 

def fine_forward_movement(k):
    global current_position_, goal_position_, state_
    initial_position = current_position_
    for i in range (1,100000):
        for j in range (1, 10):
            move_the_bot.linear.x = k*(10 - j)*(j - 1)
            move_the_bot.angular.z = 0.0
            publish_to_cmd_vel.publish(move_the_bot)
            final_position = current_position_
            print('Iteration number: ', 10*(i-1) + j, 'Distance moved:', ret_distance(initial_position, final_position))
    move_the_bot.linear.x = 0
    move_the_bot.angular.z = 0
    publish_to_cmd_vel.publish(move_the_bot)
    return 

def ret_final_orientation():
    global yaw_
    global closest_obstacle_angle_
    final_yaw_angle = (closest_obstacle_angle_+90) % 360  #Move perpendicular to the boundary
    return final_yaw_angle

def calculate_goal_angle_in_global_frame():
    global goal_position_, current_position_
    delta_x = goal_position_.x - current_position_.x
    delta_y = goal_position_.y - current_position_.y 
    beta = math.atan2(delta_y, delta_x)*180//math.pi
    beta = (beta + 360) % 360 
    return beta 

def is_reached_goal():
    global current_position_
    global goal_position_

    if ret_distance(goal_position_, current_position_) < 0.1:
        return True
    return False 

def ret_final_orientation():
    global yaw_
    global closest_obstacle_angle_
    final_yaw_angle = (closest_obstacle_angle_+90)%360  #Move perpendicular to the boundary
    return final_yaw_angle

def move_to_perp_orientation(angle):
    global yaw_
    global yaw_tol_
    global closest_obstacle_angle_
    global closest_obstacle_dist_
    final_orientation = ret_final_orientation()
    yaw_error = abs(final_orientation - yaw_) if abs(final_orientation - yaw_)<=180 else 360 - abs(final_orientation - yaw_)
    # print("final_orientation: ", final_orientation)
    # while abs(yaw_error)>= yaw_error_allowed_ or abs(closest_obstacle_angle_ - 90) < abs(closest_obstacle_angle_ - 270):
    while abs(closest_obstacle_angle_ - angle)  >= yaw_tol_:
        move_the_bot.linear.x = 0
        # move_the_bot.angular.z = 0.01*yaw_error
        move_the_bot.angular.z = 0.01*(closest_obstacle_angle_ - angle)
        publish_to_cmd_vel.publish(move_the_bot)
        final_orientation = ret_final_orientation()
        print('close angle', closest_obstacle_angle_)
        print('Moving to perp')
        # yaw_error = abs(final_orientation - yaw_) if abs(final_orientation - yaw_)<=180 else 360 - abs(final_orientation - yaw_)
        # print("turning: ", yaw_error, " closest_obstacle_angle: ", closest_obstacle_angle_)
    move_the_bot.linear.x = 0.1
    move_the_bot.angular.z = 0.0
    publish_to_cmd_vel.publish(move_the_bot)
    # print("stopped, ", yaw_error)
# Turn to align with the goal 


def action_1():
    global current_position_, goal_position_, yaw_, yaw_tol_, state_ 
    print('Entered action 1')
    beta = calculate_goal_angle_in_global_frame()
    while (abs(beta - yaw_)) >= yaw_tol_:
        move_the_bot.linear.x = 0.0
        move_the_bot.angular.z = 0.1*min((beta - yaw_), 10)
        publish_to_cmd_vel.publish(move_the_bot)
        print('Action1: Rotating, required angle = ', beta - yaw_ )
    state_ = 2
    return

# Move straight in the absence of any obstacles
def action_2():
    global state_, slow_down_dist, turn_after_slow_dist
    global closest_obstacle_angle_, closest_obstacle_dist_, dist_along_zero_
    print('Entered action 2')   
    while (closest_obstacle_dist_ >= slow_down_dist) :
        move_the_bot.linear.x = 0.1
        move_the_bot.angular.z = 0.0
        publish_to_cmd_vel.publish(move_the_bot)
        print('Action 2: Moving straight, closest obstacle straight ahead is at ', closest_obstacle_dist_)
    move_the_bot.linear.x = 0.0
    move_the_bot.angular.z = 0.0
    publish_to_cmd_vel.publish(move_the_bot)
    state_ = 3
    return 

def action_3():
    global goal_position_, current_position_, state_, angular_speed_
    global closest_obstacle_angle_, closest_obstacle_dist_
    global circ_entry_goal_dist_, circ_entry_point_, circ_exit_goal_dist_
    global circ_exit_point_, circ_start_goal_dist_, circ_start_point_
    circ_start_point_ = current_position_
    circ_start_goal_dist_ = ret_distance(goal_position_, circ_start_point_)
    while ret_distance(circ_start_point_, current_position_) < 0.1:
        print('In while 1', closest_obstacle_angle_)
        if abs(closest_obstacle_angle_ - 81) > 4: 
            print(closest_obstacle_angle_)
            if closest_obstacle_angle_ - 81 > 0:
                move_the_bot.linear.x = 0.0 
                move_the_bot.angular.z = angular_speed_ 
                publish_to_cmd_vel.publish(move_the_bot)
            else: 
                move_the_bot.linear.x = 0.0 
                move_the_bot.angular.z = -angular_speed_
                publish_to_cmd_vel.publish(move_the_bot)
        else: 
            move_the_bot.linear.x = 0.2
            move_the_bot.angular.z = 0.0
            publish_to_cmd_vel.publish(move_the_bot)
    circ_entry_point_ = current_position_
    circ_entry_goal_dist_ = ret_distance(goal_position_, circ_entry_point_)
    while ret_distance(circ_start_point_, current_position_) > 0.09: 
        print('In while 1', closest_obstacle_angle_)
        if abs(closest_obstacle_angle_ - 81) > 4: 
            print(closest_obstacle_angle_)
            if closest_obstacle_angle_ - 81 > 0:
                move_the_bot.linear.x = 0.0 
                move_the_bot.angular.z = angular_speed_ 
                publish_to_cmd_vel.publish(move_the_bot)
            else: 
                move_the_bot.linear.x = 0.0
                move_the_bot.angular.z = -angular_speed_
                publish_to_cmd_vel.publish(move_the_bot)
        else: 
            move_the_bot.angular.z = 0.0
            move_the_bot.linear.x = 0.2
            publish_to_cmd_vel.publish(move_the_bot)
        if ret_distance(current_position_, goal_position_) < ret_distance(circ_exit_point_, goal_position_):
            circ_exit_point_ = current_position_
            circ_exit_goal_dist_ = ret_distance(circ_exit_point_, goal_position_)
    state_ = 4
    move_the_bot.linear.x = 0.0
    move_the_bot.angular.z = 0.0 
    publish_to_cmd_vel.publish(move_the_bot)
    return

def action_4():
    global goal_position_, state_, current_position_
    global closest_obstacle_angle_, closest_obstacle_dist_
    global circ_entry_goal_dist_, circ_entry_point_, circ_exit_goal_dist_
    global circ_exit_point_, circ_start_goal_dist_,  circ_start_point_
    while ret_distance(circ_exit_point_, current_position_) > 0.09: 
        if abs(closest_obstacle_angle_ - 90) <= circum_yaw_tol_:
            move_the_bot.linear.x = 0.05
            move_the_bot.angular.z = 0.0
            publish_to_cmd_vel.publish(move_the_bot)
        else: 
            while abs(closest_obstacle_angle_ - 90) >= circum_yaw_tol_:
                move_the_bot.linear.x = 0.0
                move_the_bot.angular.z = 0.1
                publish_to_cmd_vel.publish(move_the_bot)
    state_ = 5
    move_the_bot.linear.x = 0.0
    move_the_bot.angular.z = 0.0 
    publish_to_cmd_vel.publish(move_the_bot)
    return


def stop():
    if ret_distance(current_position_, goal_position_) < 0.1:    
        move_the_bot.linear.x = 0
        move_the_bot.angular.z = 0
        publish_to_cmd_vel.publish(move_the_bot)
        print('Stopped ')
    return






#This is the main function and we are not making any edits to this as far as possible
if __name__ == "__main__":

    rospy.init_node('turtlebot_controller_node')
    subscribe_to_laser = rospy.Subscriber('/scan', LaserScan, callback = laser_data_callback)
    rospy.loginfo('My node has been started')
    publish_to_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    subscribe_to_odom = rospy.Subscriber('/odom', Odometry, callback = clbk_odom)
    #create an object of Twist data

    move_the_bot = Twist()
    rate_hz = 20
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        while (not wait_):
            continue
        if (state_ == 1):
            print("MAIN current state: ", state_)
            action_1()
        elif (state_ == 2):
            print("MAIN current state: ", state_)
            action_2()
        elif (state_ == 3): 
            print("MAIN current state: ", state_)
            action_3()
        elif (state_ == 4): 
            print("MAIN current state: ", state_)
            action_4()
        elif (state_ == 5 ): 
            print("MIAN current state:", state_)
            stop()

        #elif (state_ == 3):
        #    print("MAIN current state: ", state_)
        #    action_3()
        #elif (state_ == 4): 
        #    print("MAIN current state: ", state_ )
        #    action_4()
        #elif (state_ == 5): 
        #    print("MAIN current state: ", state_)
        #    action_5
        #elif (state_ == 6): 
        #    print("MAIN current state: ", state_)
        #    action_6
        
        # move_the_bot.linear.x = -1
        # move_the_bot.angular.z = 0.5
        # publish_to_cmd_vel.publish(move_the_bot)
        rate.sleep()