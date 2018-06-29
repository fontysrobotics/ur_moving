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

#include <ur3_moving/allJoints.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ur_getter");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::Publisher pubJoints = node_handle.advertise<ur3_moving::allJoints>("/move_ur/getAllJoints", 1000);
	moveit::planning_interface::MoveGroupInterface move_group("manipulator");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("manipulator");
	std::vector<double> joint_group_positions;
	while(ros::ok())
	{
		moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
		ur3_moving::allJoints joints;
		current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
		joints.joint1 = joint_group_positions[0];
		joints.joint2 = joint_group_positions[1];
		joints.joint3 = joint_group_positions[2];
		joints.joint4 = joint_group_positions[3];
		joints.joint5 = joint_group_positions[4];
		joints.joint6 = joint_group_positions[5];
		
		pubJoints.publish(joints);

	}

	spinner.stop();

	ros::shutdown();
	return 0;
}
