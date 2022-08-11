/** @package rt2_assignment2
* 
*   @file state_machine.cpp
*   @brief This file implements the FSM
*
*   @author Riccardo Zuppetti
*   @version 1.0
*   @date 06/06/2022
*   @details
*   
*   Subscribes to: <BR>
*	    None
*
*   Publishes to: <BR>
*       /time
*       /reach
*
*   Services: <BR>
*       /user_interface
* 
*   Client: <BR>
*       /position_server
*
*   Action Client: <BR>
*       /go_to_point
*
*   Description: <BR>
*       This node is a server for the user interface.
*       It will receive commands from the user interface.
*       It can elaborate new random target from the position_server.
*       It can start or stop the robot motion with the go_to_point action.
*/

#include "ros/ros.h"
#include "rt2_assignment2/Command.h"
#include "rt2_assignment2/RandomPosition.h"
#include <rt2_assignment2/PlanningAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

bool start = false; // boolean that depends on the the command received
bool moving = false; // boolean to know if the robot is moving or not
std_msgs::Bool reached;	// to state if the goal has been achieved or not

/**
 *  @brief: It receives the commands from the user_interface node
 *  @param req: the command received from the client
 *  @param res: not set
 * 
 *  @return: true
 * 
 *  This function sets the global variable start to true if the command 
 *  received is "start" or it sets it to false if the command is different
 */

bool user_interface(rt2_assignment2::Command::Request &req, rt2_assignment2::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}

/**
 *  @brief: The main function
 * 
 *  @return: 0
 * 
 *  This function initializes the ros node, the server, the clients.
 *  Then if ros is running it checks the global variable start. If start 
 *  is true and the robot is not already moving a new goal is set.
 *  If the robot is already moving then it is checked if the goal is reached. 
 *  If the global variable start is false and the robot is moving then
 *  all the goals will be canceled.
 */


int main(int argc, char **argv)
{
   // initialization of the node
	ros::init(argc, argv, "state_machine");
	ros::NodeHandle n;
	ros::NodeHandle n1;
	ros::NodeHandle n2;
	ros::NodeHandle n3;
	//initialization of the server
	ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
	ros::ServiceClient client_rp = n1.serviceClient<rt2_assignment2::RandomPosition>("/position_server");
	actionlib::SimpleActionClient<rt2_assignment2::PlanningAction> ac("/go_to_point");
	ros::Publisher pub=n2.advertise<std_msgs::Bool>("/reach", 1000);
    ros::Publisher pub_time=n3.advertise<std_msgs::Float32>("/time", 1000);
   
    rt2_assignment2::RandomPosition rp;
    rp.request.x_max = 5.0;
    rp.request.x_min = -5.0;
    rp.request.y_max = 5.0;
    rp.request.y_min = -5.0;

    auto start_time=std::chrono::steady_clock::now();
    auto end_time=std::chrono::steady_clock::now();
    float elapsed_time;
    std_msgs::Float32 time_total;
   
   while(ros::ok()){
        ros::spinOnce();
   	    if (start){
            // the user requested to move
            if(!moving){
                // the robot is not moving
                // goal position requested
   		        client_rp.call(rp);
                rt2_assignment2::PlanningGoal goal;
			    goal.target_pose.header.frame_id = "base_link";
			    goal.target_pose.header.stamp = ros::Time::now();
			    goal.target_pose.pose.position.x = rp.response.x;
		        goal.target_pose.pose.position.y = rp.response.y;
			    goal.target_pose.pose.orientation.z = rp.response.theta;
			    std::cout << "\nGoing to the position: x= " << rp.response.x << " y= " <<rp.response.y << " theta = " <<rp.response.theta << std::endl;
			    ac.sendGoal(goal);
                start_time=std::chrono::steady_clock::now();
		        moving = true;
            }
            else{
                // the robot is already moving
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    // checked if the goal has been achieved
				    std::cout << "Goal completed" << std::endl;
					moving = false;
                    reached.data=true;
                    pub.publish(reached);
                    end_time=std::chrono::steady_clock::now();
                    std::chrono::duration<double> elapsed_seconds = end_time-start_time;
					std::cout <<"elapsed time:" <<elapsed_seconds.count() <<"s\n";
					elapsed_time=float(elapsed_seconds.count());
					time_total.data=elapsed_time;
					pub_time.publish(time_total);

				}
            }
   	    }
        else{
            // the user requested to stop the robot
            if(moving){
                // the robot is already moving
            	ac.cancelAllGoals(); / all goals are canceled
				std::cout << "Goal cancelled" << std::endl;
				moving = false;
                reached.data=false;
				pub.publish(reached);
            }
        }
   }
   return 0;
}