#! /usr/bin/env python
"""
.. module:: go_to_point
    :platform: Unix
    :synopsis: This file implements the behaviour that allows the robot to reach a goal.
.. moduleauthor:: Riccardo Zuppetti

This node allows the robot to reach a position with a given orientation.
Firstly it orients the robot in the direction of the goal and moves towards it.
Once the robot has reached the correct x and y coordinates it rotates to reach
the correct orientation. The velocities, both angular and linear are set by
the node *user_interface* and are received on the topic */vel*. If the goal
is set cancelled by the client of the action server then the velocities are
all set to zero and the action server is set preempted.  
 
Subscribes to:

/odom topic where the simulator publishes the robot position

/vel where the *user_interface* publishes the requested velocities

Publishes to:

/cmd_vel the desired robot velocity, it depends on the state

Service :

/go_to_point to start the robot motion.
"""

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib
import rt2_assignment2.msg

# robot state variables
position_ = Point()
"""Point: actual robot position
"""
yaw_ = 0
"""Float: actual robot angle
"""
position_ = 0
state_ = 0
"""Int: state of the server
"""
pub_ = None
"""To publish on the */cmd_vel* topic
"""

# Auxiliary variables
des_position = Point()
"""Point: desired robot position
"""
des_position.z = 0
"""Float: desired robot orientation
"""
temp_flag = False
"""Bool: to store wheater the goal has been reached
"""

# parameters for control
yaw_precision_ = math.pi / 9
"""Float: +/- 20 degrees of precision allowed
"""
yaw_precision_2_ = math.pi / 90
"""Float: +/- 2 degrees of precision allowed
"""
dist_precision_ = 0.1
"""Float: precision of the linear distace allowed
"""
kp_a = -3.0
"""Float: coefficent
"""
kp_d = 0.2
"""Float: coefficent
"""
ub_a = 0.6
"""Float: coefficent
"""
lb_a = -0.5
"""Float: coefficent
"""
ub_d = 0.6
"""Float: coefficent
"""
vel_ = Twist()
"""Twist: the requested velocities
"""

# parameter for action
action_server = None
"""To initialize the action server
"""

def clbk_vel(msg):
    """
    This function saves the data received by the subscriber on the topic */vel*
    in a global variable of type Twist, Vel, this variable will be used when
    there is the need to set a new velocity.
    
    Args :
        msg(Twist): the data received on the topic */vel*
    
    Returns :
        None
    """
    global vel_
    vel_.linear.x=msg.linear.x
    vel_.angular.z=msg.angular.z

def clbk_odom(msg):
    """
    This function saves the data received by the subscriber on the topic */odom*
    in the global variable position for the information about the current position
    of the robot. It then changes the format of the orientation from quaternions angles
    to euler angles; it is the extracted the third element of the vector and it is saved
    on the global variable *yaw_*
    
    Args :
        msg(Odometry): the data received on the topic */odom*
    
    Returns :
        None
    """
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

def change_state(state):
    """
    This function receives the new state and assigns its value to the global 
    variable *states_*. Then a message is printed to know which one is the
    new state.
    
    Args :
        state(Int): the state of the robot
    
    Returns :
        None
    """
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

def normalize_angle(angle):
    """
    This function normalizes the angle received as input, it doesn't change 
    the sign but it reduces the magnitude to less than one full circle
    
    Args :
        angle(Float): the angle I want to normalize
    
    Returns :
        angle(Float): the normalized angle.
    """
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    """
    This function calculates the desired orientation to reach the x,y point
    and set the angular velocity to reach it. If the orientation error is 
    less than a given threshold then the state is changed to the behaviour
    go straight.
   
    Args :
        des_pos(Point): the desired x and y coordinates
    
    Returns :
        None
    """
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


def go_straight_ahead(des_pos):
    """
    This function calculates the desired orientation to reach the x,y point and the distance between the goal both linear and angular.
    It then sets the linear velocity. It also set an angular velocity proportional to the error to slightly correct the direction of the line when needed.
    If the distance between the goal is less than a given threshold the state is changed to the fix final orientation behaviour.
    
    Args :
        des_pos(Point): the desired x and y coordinates
    
    Returns :
        None
    """
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


def fix_final_yaw(des_yaw):
    """
    This function calculates the error between the current orientation and the 
    desired one. It then sets the angular velocity to obtain the correct orientation.
    If the error is less than a given threshold then the state is changed to done.
    
    Args :
        des_yaw(Float): the desired orientation
    
    Returns :
        None
    """
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
        
def done():
    """
    This function puts to zero all the velocities, angular and linear, and sets
    the goal as succeeded.  
    
    Args :
        None
    
    Returns :
        None
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
    action_server.set_succeeded()
    temp_flag = True
 

def go_to_point(goal):
    """
    This function is called when a request to the server is made. It sets 
    all the global variables needed, then it enteres a while loop. In the while
    loop it always check if the goal is preempted and if that is the case it
    sets all the velocities to zero and it set the goal as preempted. If 
    the action server is not preempted it checks which is the state and it 
    calls the corresponding function.  
    
    Args :
        goal: the desired position and orientation to obtain
    
    Returns :
        None
    """
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


def main():
    """
    This function is called when the program is first requested to run. It
    initializes all the publishers, subscribers, services and then waits for
    a request for the action server that should come from the user_interface
    node.
    
    Args :
        None
    
    Returns :
        None
    """
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
