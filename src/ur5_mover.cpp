#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  sleep(2.0);
  
  moveit::planning_interface::MoveGroup group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
  geometry_msgs::Pose start_state = group.getCurrentPose().pose;
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.7;
  target_pose1.position.z = 1.0;
  group.setPoseTarget(target_pose1);


  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();

  collision_object.id = "box1";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 2;
  primitive.dimensions[2] = 1;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -1.1;
  box_pose.position.y = 0;
  box_pose.position.z = 0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;  
  collision_objects.push_back(collision_object); 
  ROS_INFO("Add an object into the world");  
  planning_scene_interface.addCollisionObjects(collision_objects);

  collision_object.id = "box2";
  //shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 2;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 1;

 //geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0;
  box_pose.position.y = 1.1;
  box_pose.position.z = 0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  collision_objects.push_back(collision_object); 
  ROS_INFO("Add an object into the world");  
  planning_scene_interface.addCollisionObjects(collision_objects);

  collision_object.id = "box3";
  //shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 2;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 1;

 //geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0;
  box_pose.position.y = -1.1;
  box_pose.position.z = 0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  collision_objects.push_back(collision_object); 
  ROS_INFO("Add an object into the world");  
  planning_scene_interface.addCollisionObjects(collision_objects);
  sleep(2.0);
  moveit::planning_interface::MoveGroup::Plan my_plan;
  /*
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success;
  int i =0;
  do{
    success = group.plan(my_plan);
    i++;
    if (success)
      break;
  }while( i < 5);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
  sleep(10.0); */
  /*
  if (1)
  {
    ROS_INFO("Visualizing plan 1 (again)");    
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    sleep(10.0);
  }*/
  
  /* group.move() */

  // JOINT SPACE GOAL
 /*
  
  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  
  group_variable_values[0] = -1.0;  
  group.setJointValueTarget(group_variable_values);
  success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
  
  sleep(10.0); 

  /*
  moveit_msgs::OrientationConstraint ocm;  
  ocm.link_name = "base_link";  
  ocm.header.frame_id = "base_link";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;
  
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);  
  group.setPathConstraints(test_constraints); 

  
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  const robot_state::JointModelGroup *joint_model_group =
                  start_state.getJointModelGroup(group.getName());
  start_state.setFromIK(joint_model_group, start_pose2);
  group.setStartState(start_state);
  
  // Now we will plan to the earlier pose target from the new 
  // start state that we have just created.
  group.setPoseTarget(target_pose1);
  success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
  sleep(10.0); 
  group.clearPathConstraints();
  */
  /*
  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose3 = start_state;
   //float xa = start_state.position.x - 0.6;
   //float za = start_state.position.z - 0.6;
   //float ya = start_state.position.y - 0.6;

   //for(int i = 0; i<(int)(xa/0.02); i++){

  target_pose3.position.x = 0.4;
  target_pose3.position.z = 0.2;
  target_pose3.position.y = 0.6;
  waypoints.push_back(target_pose3);  
  //}  
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);

  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);
  sleep(15.0);
  */
 
/*
  // Now, let's add the collision object into the world
  
  
  sleep(2.0);

  group.setPlanningTime(10.0);
  group.setStartState(*group.getCurrentState());
  group.setPoseTarget(target_pose1);
  success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 5 (pose goal move around box) %s",
    success?"":"FAILED");
  sleep(10.0);
  
  ROS_INFO("Attach the object to the robot");  
  group.attachObject(collision_object.id);  
  sleep(4.0);


  ROS_INFO("Detach the object from the robot");  
  group.detachObject(collision_object.id);  
  sleep(4.0);


  ROS_INFO("Remove the object from the world");  
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);  
  planning_scene_interface.removeCollisionObjects(object_ids);
  sleep(4.0);
*/

  

  ros::shutdown();  
  return 0;
}