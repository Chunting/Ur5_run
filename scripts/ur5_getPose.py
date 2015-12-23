#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String

def move_group_python_interface_tutorial():
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("manipulator")
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)
  print "============ Waiting for RVIZ..."
  rospy.sleep(10)
  print "============ Starting tutorial "
  print "============ Reference frame: %s" % group.get_planning_frame()
  grip = group.get_end_effector_link()
  print "============ Reference frame: %s" % group.get_end_effector_link()
  print "============ Robot Groups:"
  print robot.get_group_names()
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"
  curr = group.get_current_pose(grip).pose
  print curr
  group.set_goal_position_tolerance(0.01)
  group.set_goal_orientation_tolerance(0.1)
  group.allow_replanning(True)
  rospy.sleep(5)
  fraction = 0.0
  attempts = 0
  maxAttempt = 100
  waypoints = []
  waypoints.append(group.get_current_pose(grip).pose)
  wpose = geometry_msgs.msg.Pose()
  wpose.orientation.w = 0.33
  wpose.position.x =0 
  wpose.position.y = waypoints[0].position.y
  wpose.position.z = waypoints[0].position.z
  waypoints.append(copy.deepcopy(wpose))
  group.set_start_state_to_current_state()
  while fraction < 1.0 and attempts < maxAttempt:
	  (plan3, fraction) = group.compute_cartesian_path(
		                       waypoints,   # waypoints to follow
		                       0.01,        # eef_step
		                       0.0)         # jump_threshold
	  attempts += 1
          print attempts
	  #rospy.sleep(1)
                               
  print "============ Waiting while RVIZ displays plan3..." + str(fraction)
  rospy.sleep(15)
  if fraction == 1.0:
        print "executin" + str(plan3)
        d = 1
        for x in plan3.joint_trajectory.points:
		x.velocities = [0.0,0.0,0.0,0.0,0.0,0.0]
		time_from_start=rospy.Duration(d)
 		d += 1
        print "executin2" + str(plan3)
  	group.execute(plan3)

  while True:
  	curr = group.get_current_pose()
  	print curr
	rospy.sleep(5)


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
