#pragma once

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

namespace ur3_teleoperation{

class CollisionObjects{
public:
	CollisionObjects(ros::NodeHandle& nh, std::string planning_group);
	virtual ~CollisionObjects();

	void addBox(std::string object_id,	double box_size[3], double box_position[3]);

	moveit::planning_interface::PlanningSceneInterface planning_scene_;
	moveit::planning_interface::MoveGroupInterface *move_group_ptr_;
	const robot_state::JointModelGroup* joint_model_group_;
	std::vector<moveit_msgs::CollisionObject> collision_objects_;

private:
	ros::NodeHandle nh_;
};
} // Namespace
