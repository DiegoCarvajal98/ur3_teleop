#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <ur3_teleoperation/CollisionObjects.hpp>

namespace ur3_teleoperation{

CollisionObjects::CollisionObjects(ros::NodeHandle& nh, std::string planning_group) :
 nh_(nh){
	move_group_ptr_ = new moveit::planning_interface::MoveGroupInterface(planning_group);
	joint_model_group_ = move_group_ptr_->getCurrentState()->getJointModelGroup(planning_group);

	double table_dimensions[3] = {1, 0.75, 0.69};
	double table_pose[3] = {0, 0.375, -0.36};

	double base_dimensions[3] = {0.5, 0.5, 0.69};
	double base_pose[3] = {0, -0.11, -0.345};

	addBox("table", table_dimensions, table_pose);
	addBox("base", base_dimensions, base_pose);

	planning_scene_.addCollisionObjects(collision_objects_);

	ros::shutdown();
}

void CollisionObjects::addBox(std::string object_id, double box_size[3],
		double box_position[3]){

	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = move_group_ptr_->getPlanningFrame();
	collision_object.id = object_id;

	shape_msgs::SolidPrimitive box;
	box.type = box.BOX;
	box.dimensions.resize(3);
	box.dimensions[0] = box_size[0];
	box.dimensions[1] = box_size[1];
	box.dimensions[2] = box_size[2];

	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x = box_position[0];
	box_pose.position.y = box_position[1];
	box_pose.position.z = box_position[2];

	collision_object.primitives.push_back(box);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	ROS_INFO_STREAM("Collision object '" << object_id << "' added");

	collision_objects_.push_back(collision_object);
}

CollisionObjects::~CollisionObjects(){
}

}
