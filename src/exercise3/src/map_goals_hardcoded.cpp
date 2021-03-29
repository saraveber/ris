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

using namespace std;
using namespace cv;

float x [13] = {-0.4111142158508301, -0.24434518814086914, -1.0020184516906738, -0.06022310256958008, 1.3457698822021484,1.9823939800262451,2.8807902336120605,
				3.1241297721862793,2.311507225036621,1.5406930446624756,1.9404168128967285,2.5390520095825195,1.167187213897705}; 
float y [13] = {-2.903233051300049, -1.6592564582824707, -1.1628522872924805, 0.3249940872192383, 0.3903846740722656,-0.3256950378417969,-0.5152130126953125,
				-1.1503229141235352,-2.30399227142334,-1.4829859733581543,-3.2899951934814453,-3.831695556640625,-3.7933597564697266};


geometry_msgs::TransformStamped map_transform;



int main(int argc, char** argv){
	ros::init(argc, argv, "map_goals_hardcoded");

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;
	
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.orientation.w = 1;
	
	//goal.target_pose.pose.position.x = 2.0;
    //goal.target_pose.pose.position.y = 1.0;
    //goal.target_pose.pose.orientation.w = 0.0;
    
    
    for (int i = 0; i < 13; i++){
    	goal.target_pose.pose.position.x = x[i];
    	goal.target_pose.pose.position.y = y[i];
    	ROS_INFO("Sending goal");
		ac.sendGoal(goal);

		//ac.waitForResult(ros::Duration(15.0));
		ac.waitForResult();

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("You arived at destination: %d.",(i+1));
		else
			ROS_INFO("UPSI, you are stuck.");

		move_base_msgs::MoveBaseGoal goal1;

		goal1.target_pose.header.frame_id = "base_link";
		goal1.target_pose.header.stamp = ros::Time::now();
		goal1.target_pose.pose.position.y = 0;
		goal1.target_pose.pose.orientation.z = 20;
		goal1.target_pose.pose.orientation.w = 1;

		
		ROS_INFO("Sending goal");
	  	ac.sendGoal(goal1);

		ac.waitForResult();

	  	move_base_msgs::MoveBaseGoal goal2;
		

		goal2 = goal1;
		goal2.target_pose.header.stamp = ros::Time::now();
		

		
		ROS_INFO("Sending goal");
		
	  	ac.sendGoal(goal2);

		ac.waitForResult();

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		   ROS_INFO("Hooray, the base moved 1 meter forward");
	  	else
	  		ROS_INFO("The base failed to move forward 1 meter for some reason");

				
	    }

	return 0;
}
