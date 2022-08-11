#! /usr/bin/env python

##  \package rt2_assignment2
#
#   \file go_to_point.py
#   \brief This script allows to the robot to reach a random goal position
#   
#   \author Riccardo Zuppetti
#   \version 1.0
#   \date 06/06/2022
#   \details
#  
#   Subscribes to: <BR>
#       /odom
#       /vel
#
#   Publishes to: <BR>
#       /cmd_vel 
#
#   Services: <BR>
#       None
#
#   Action Services: <BR>
#       /go_to_point
#
#   Description: <BR>
#       This node allows the robot to reach a random position with a given orientation.
#       To reach this position, the robot orients itself in the direction of the goal position.
#       Then the robot moves towards the goal in a straight direction.
#       Reached the position, the robots rotates itself to achieve the orientation provided.
#       It subscribes to the /odom topic in order to retrieve the current pose of the robot
#       and publishes the velocities on the /cmd_vel topic.

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib
import rt2_assignment2.msg

# robot state variables
position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None

# Auxiliary variables
des_position = Point()
des_position.z = 0
temp_flag = False

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6
vel_ = Twist()

# parameter for action
action_server = None

##
#	\brief This function is called when new data are available on the topic /vel
#	\param msg: the data received on the topic /vel
#	\return : None
# 	
#	This function saves the data received by the subscriber on the topic /vel
#	in a global variable of type Twist, Vel, this variable will be used when
#	there is the need to set a new velocity.

def clbk_vel(msg):
    global vel_
    vel_.linear.x=msg.linear.x
    vel_.angular.z=msg.angular.z

##
#   \brief This function is called when new data are available on the topic /odom
#   \param msg: the data received on the topic /odom
#	\return : None
# 	
#	This function saves the data published on the /odom topic to retrieve the current
#   pose of the robot (position & orientation). It then changes the format of the orientation
#   from quaternions angles to euler angles.

def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

##
#   \brief This function changes the state
#	\param state: the state of the robot
#	\return : None
# 	
#	This function receives the new state and assigns its value to the global 
#	variable states_.

def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

##
#	\brief This function normalizes angles
#	\param angle: the angle to norrmalize
#	\return : the normalized angle
# 	
#	This function normalizes the angle received as input.

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

##
#	\brief This function orients the robot towards the goal
#	\param des_pos: the desired position to be reached
#	\return : None
# 	
#   It rotates the robot to fix the yaw in the desired direction
#   by publishing a twist message on /cmd_vel

def fix_yaw(des_pos):
    global yaw_, pub_, yaw_precision_2_, state_, vel_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = vel_.angular.z
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = vel_.angular.z
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


##
#	\brief This function moves the robot in a straight line
#	\param des_pos: the desired position to be reached
#	\return : None
# 	
#   This function allows to the robot to go straight
#   or to change the state if needed

def go_straight_ahead(des_pos):
    global yaw_, pub_, yaw_precision_, state_, vel_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = vel_.linear.x
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = vel_.linear.x

        twist_msg.angular.z = vel_.angular.z*err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        # print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        # print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

##
#	\brief This function orients the robot in the goal position
#	\param des_yaw: the desired orientation
#	\return : None
# 	
#	This function calculates the error between the current orientation and the 
#	desired one. It then sets the angular velocity to obtain the correct orientation.
#	If the error is less than a given threshold then the state is changed to done.

def fix_final_yaw(des_yaw):
    global vel_
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = vel_.angular.z
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = vel_.angular.z
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = vel_.angular.z
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)

##
#	\brief This function stops the robot
#	\param : None
#	\return : None
# 	
#	This function puts to zero all the velocities, angular and linear, and sets
#	the goal as succeeded.  
        
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
    action_server.set_succeeded()
    temp_flag = True

##
#	\brief This function implements the go_to_point behaviour
#	\param goal: the desired position and orientation to reach
#	\return : None
# 	
#   This function is called when a request to the server is made. It sets 
#	all the global variables needed, then it enteres a while loop. In the while
#	loop it always check if the goal is preempted and if that is the case it
#	sets all the velocities to zero and it set the goal as preempted. If 
#	the action server is not preempted it checks which is the state and it 
#	calls the corresponding function.     

def go_to_point(goal):
    global state_, des_position, action_server, temp_flag
    des_position.x = goal.target_pose.pose.position.x
    des_position.y = goal.target_pose.pose.position.y
    des_yaw = goal.target_pose.pose.position.z
    change_state(0)
    while True:
        # if the action is preempted
        if action_server.is_preempt_requested():
            rospy.loginfo('Goal was preempted')
            # publish a null velocity
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            pub_.publish(twist_msg)
            # goal preempted
            action_server.set_preempted()
            temp_flag = False
            break
        elif state_ == 0:
            # orientation towards the goal
            fix_yaw(des_position)  
        elif state_ == 1:
            # moving in straight direction
            go_straight_ahead(des_position)   
        elif state_ == 2:
            # final orientation
            fix_final_yaw(des_yaw) 
        elif state_ == 3:
            # goal reached
            done()
            break
    return True

##
#	\brief This function implements the ros node
#	\param : None
#	\return : None
# 	
#	This function is called when the program is first requested to run. It
#	initializes all the publishers, subscribers, services and then waits for
#	a request for the action server that should come from the user_interface node.

def main():
    global pub_, action_server
    # initialize the goal
    rospy.init_node('go_to_point')
    # publish the velocity
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # subscriber to the /odom topic
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub_vel = rospy.Subscriber('/vel', Twist, clbk_vel)
    # initialize the action server
    action_server = actionlib.SimpleActionServer('/go_to_point', rt2_assignment2.msg.PlanningAction, go_to_point, auto_start=False)
    action_server.start()
    rospy.spin()

if __name__ == '__main__':
    main()