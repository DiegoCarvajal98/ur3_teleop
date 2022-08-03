#include <ros/ros.h>
#include <ur3_teleoperation/CollisionObjects.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char** argv){
	/* Add collision objects node
	ros::init(argc, argv, "add_collision_objects");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroupInterface move_group("arm");
	moveit::planning_interface::PlanningSceneInterface planning_scene;
	const robot_state::JointModelGroup* joint_model_group =
			move_group.getCurrentState()->getJointModelGroup("arm");

	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = move_group.getPlanningFrame();
	collision_object.id = "table";

	shape_msgs::SolidPrimitive table;
	table.type = table.BOX;
	table.dimensions.resize(3);
	table.dimensions[0] = 0.913;
	table.dimensions[1] = 0.913;
	table.dimensions[2] = 0.775;

	geometry_msgs::Pose table_pose;
	table_pose.orientation.w = 1.0;
	table_pose.position.x = -0.2;
	table_pose.position.y = 0.7;
	table_pose.position.z = -0.2875;

	collision_object.primitives.push_back(table);
	collision_object.primitive_poses.push_back(table_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);

	planning_scene.addCollisionObjects(collision_objects);

	ROS_INFO_STREAM("Collision object added");

	std::vector<std::string> objects;
	objects = planning_scene.getKnownObjectNames();

	for (std::string i : objects){
		ROS_INFO_STREAM("Added objects: " << i);
	}

	ros::shutdown();
	return 0;
	*/

	ros::init(argc, argv, "add_collision_objects");
	ros::NodeHandle nh("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	std::string planning_group = "arm";
	ur3_teleoperation::CollisionObjects add_collision_objects(nh, planning_group);

	return 0;
}
