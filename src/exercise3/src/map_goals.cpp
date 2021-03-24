#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;




int main(int argc, char** argv){
	ros::init(argc, argv, "map_goals");

	  
	MoveBaseClient ac("move_base", true);

	while(!ac.waitForServer(ros::Duration(5.0))){
	    ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.y = 0;
	goal.target_pose.pose.orientation.z = 0;
	goal.target_pose.pose.orientation.w = 1;

	
	ROS_INFO("Sending goal");
  	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	   ROS_INFO("Hooray, the base moved 1 meter forward");
  	else
  		ROS_INFO("The base failed to move forward 1 meter for some reason");
	
 
  	
  	return 0;
}