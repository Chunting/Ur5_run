#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
	ros::init (argc, argv, "move_group_tutorial");
  	ros::AsyncSpinner spinner(1);
  	spinner.start();
  	ros::NodeHandle node_handle;

	moveit::planning_interface::MoveGroup group("manipulator");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	ROS_INFO("I heard2: ");
	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
	return 0;
}