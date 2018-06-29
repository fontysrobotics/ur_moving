#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

#include <ur3_moving/singleJoint.h>
#include <ur3_moving/allJoints.h>
#include <ur3_moving/endEffector.h>

void setEndEffector(const ur3_moving::endEffector::ConstPtr& msg)
{
	ros::AsyncSpinner spinner(1);
	spinner.start();
	moveit::planning_interface::MoveGroupInterface move_group("manipulator");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("manipulator");

	int success = 0;

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	move_group.setPoseTarget(msg->pose);

	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if(success)
	{
		ROS_INFO("Success in planning!");  
		if(msg->planOnly)
		{
			ROS_INFO("Only planned.");
		}
		else
		{
			bool move = false;
			if(msg->confirmation)
			{
				ROS_INFO("Do you want to move it? (y/N)");
				char input[64];
				std::cin >> input;
				if(input[0] == 'y')
				{
					ROS_INFO("Going to move!");
					move = true;
				}
				else
				{
					move = false;
				}
			}
			else
			{
				move = true;
			}
			
			if(move)
			{
				move_group.move();
			}
			else
			{
				ROS_INFO("Not going to move.");
			}
		}
	}
	else
	{
		ROS_INFO("Planning failed....");
	}
	spinner.stop();
	
}

void addToJoint(const ur3_moving::singleJoint::ConstPtr& msg)
{
	ros::AsyncSpinner spinner(1);
	spinner.start();
	moveit::planning_interface::MoveGroupInterface move_group("manipulator");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("manipulator");

	int success = 0;

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	joint_group_positions[msg->joint] += msg->radians;

	move_group.setJointValueTarget(joint_group_positions);

	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if(success)
	{
		ROS_INFO("Success in planning!");  
		if(msg->planOnly)
		{
			ROS_INFO("Only planned.");
		}
		else
		{
			bool move = false;
			if(msg->confirmation)
			{
				ROS_INFO("Do you want to move it? (y/N)");
				char input[64];
				std::cin >> input;
				if(input[0] == 'y')
				{
					ROS_INFO("Going to move!");
					move = true;
				}
				else
				{
					move = false;
				}
			}
			else
			{
				move = true;
			}
			
			if(move)
			{
				move_group.move();
			}
			else
			{
				ROS_INFO("Not going to move.");
			}
		}
	}
	else
	{
		ROS_INFO("Planning failed....");
	}
	spinner.stop();
}

void moveJointAbsolute(const ur3_moving::singleJoint::ConstPtr& msg)
{
	ros::AsyncSpinner spinner(1);
	spinner.start();
	moveit::planning_interface::MoveGroupInterface move_group("manipulator");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("manipulator");

	int success = 0;

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	joint_group_positions[msg->joint] = msg->radians;

	move_group.setJointValueTarget(joint_group_positions);

	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if(success)
	{
		ROS_INFO("Success in planning!");  
		if(msg->planOnly)
		{
			ROS_INFO("Only planned.");
		}
		else
		{
			bool move = false;
			if(msg->confirmation)
			{
				ROS_INFO("Do you want to move it? (y/N)");
				char input[64];
				std::cin >> input;
				if(input[0] == 'y')
				{
					ROS_INFO("Going to move!");
					move = true;
				}
				else
				{
					move = false;
				}
			}
			else
			{
				move = true;
			}
			
			if(move)
			{
				move_group.move();
			}
			else
			{
				ROS_INFO("Not going to move.");
			}
		}
	}
	else
	{
		ROS_INFO("Planning failed....");
	}
	spinner.stop();
}

void moveAllJoints(const ur3_moving::allJoints::ConstPtr& msg)
{
	ros::AsyncSpinner spinner(1);
	spinner.start();
	moveit::planning_interface::MoveGroupInterface move_group("manipulator");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("manipulator");

	int success = 0;

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	
	joint_group_positions[0] = msg->joint1;
	joint_group_positions[1] = msg->joint2;
	joint_group_positions[2] = msg->joint3;
	joint_group_positions[3] = msg->joint4;
	joint_group_positions[4] = msg->joint5;
	joint_group_positions[5] = msg->joint6;
	
	move_group.setJointValueTarget(joint_group_positions);

	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if(success)
	{
		ROS_INFO("Success in planning!");  
		if(msg->planOnly)
		{
			ROS_INFO("Only planned.");
		}
		else
		{
			bool move = false;
			if(msg->confirmation)
			{
				ROS_INFO("Do you want to move it? (y/N)");
				char input[64];
				std::cin >> input;
				if(input[0] == 'y')
				{
					ROS_INFO("Going to move!");
					move = true;
				}
				else
				{
					move = false;
				}
			}
			else
			{
				move = true;
			}
			
			if(move)
			{
				move_group.move();
			}
			else
			{
				ROS_INFO("Not going to move.");
			}
		}
	}
	else
	{
		ROS_INFO("Planning failed....");
	}
	spinner.stop();
}

void printEndEffector(const std_msgs::String::ConstPtr& msg)
{
	ros::AsyncSpinner spinner(1);
	spinner.start();
	moveit::planning_interface::MoveGroupInterface move_group("manipulator");
	
	geometry_msgs::PoseStamped pose = move_group.getCurrentPose();
	ROS_INFO_STREAM("Pose: " << pose);
	spinner.stop();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ur_setter");
	ros::NodeHandle node_handle;
	ros::Subscriber sub = node_handle.subscribe("/move_ur/addToJoint", 1000, addToJoint);
	ros::Subscriber submove = node_handle.subscribe("/move_ur/moveSingleJoint", 1000, moveJointAbsolute);
	ros::Subscriber submoveAll = node_handle.subscribe("/move_ur/moveAllJoints", 1000, moveAllJoints);
	ros::Subscriber subsetEnd = node_handle.subscribe("/move_ur/setEndEffector", 1000, setEndEffector);
	ros::Subscriber subGetPose = node_handle.subscribe("/move_ur/printEndEffector", 1000, printEndEffector);
	while(ros::ok())
	{
		ros::spinOnce();
	}

	ros::shutdown();
	return 0;
}
