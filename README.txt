Goal: Goal of the project was to position the UR5 arm based on the input.

Things done:
1. Installation and understanding fo move it
2. Planning a path of the endeffector using default planner
3. Subscribing to position topic to get position 
4. moving to the position

Future work:
1. Finding position of the an object
2. publishing the position



Installation:
1. Install Ros Indigo
2. Install moveit from source code - Debian file gives an error very difficult to trace back
	source /opt/ros/indigo/setup.bash
	mkdir moveit
	cd moveit
	mkdir src
	cd src/
	wstool init .
	wstool merge https://raw.github.com/ros-planning/moveit_docs/indigo-devel/moveit.rosinstall
	wstool update
	cd ..
	catkin_make

3. Create Ur5_moveit_config package:

Note: You can find complete project with required dependency files at http://www.github.com/amayakal/ur5_moveit_config

	1. roslaunch moveit_setup_assistant setup_assistant.launch
	2. Create a new package and select the path to your robot's URDF file
	3. Calculate Self-Collision matrix (~80,000?)
	4. Add required Virtual Joint:
		moveIt requires at least one virtual joint, connecting the robot to the world
		Add Virtual Joint:
		name = 'FixedBase' (arbitrary)
		child = 'world' (should match the URDF root link)
		parent = 'world' (arbitrary, until robot is placed in an environment)
		`type = 'fixed'
	5. Add Planning Groups:
		Add manipulator (arm) group
		Add Group, name = 'manipulator', kinematic solver = 'KDLKinematicsPlugin'
		Add Kin. Chain
			assign base_link and tip_link (e.g. link_6, tool0, or your robot's equivalent)
	6. Add end-effector group [OPTIONAL]
		If you define an End-Effector, moveIt can use this in grasp-planning and visualization methods.
		If your robot doesn't have an end-effector, just leave this blank.
		Add Group, name = 'gripper', kinematic solver = 'none'
		Add Links
		assign end-effector links
	7. Add Robot Poses [OPTIONAL]:
		Add one or more default poses, to reference in later planning code
		(all-zeros, home, etc.)
		The MoveIt! button will cycle the virtual robot through all defined poses
		Add an End Effector [OPTIONAL]:
		name = 'gripper' (arbitrary)
		group = 'gripper' (should match group created above)
		parent = 'tool0' (last arm link)
		parent group = 'manipulator'
	8. Generate Configuration Files
	9. Create the controllers.yaml file (<robot>_moveit_config/config/controllers.yaml):
		controller_list:
		  - name: ""
		    action_ns: joint_trajectory_action
		    type: FollowJointTrajectory
		    joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
	10. Create the joint_names.yaml file (<robot>_moveit_config/config/joint_names.yaml):
			controller_joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
	11. Fill in the blank controller_manager launch file (<robot>_moveit_config/launch/<robot>_moveit_controller_manager.launch):
			<launch>
			  <arg name="moveit_controller_manager"
			       default="moveit_simple_controller_manager/MoveItSimpleControllerManager"/>
			  <param name="moveit_controller_manager"
			         value="$(arg moveit_controller_manager)"/>

			  <rosparam file="$(find ur5_moveit_config)/config/controllers.yaml"/>
			</launch>
	12. Ceate the Planning & Execution launch file:
			Create ur5_moveit_config/launch/moveit_planning_execution.launch using file on github as template
5. cd ~/moveit
	 git clone https://github.com/ros-industrial/industrial_core
	 catkin_make
	 roslaunch ur5_moveit_config moveit_planning_execution.launch


4. Motion planner
	Create IK fast solution
	http://wiki.ros.org/Industrial/Tutorials/Create_a_Fast_IK_Solution
	Create moveit plugin
	http://wiki.ros.org/Industrial/Tutorials/Create_a_Fast_IK_Solution/moveit_plugin

Kinematics:
https://smartech.gatech.edu/bitstream/handle/1853/50782/ur_kin_tech_report_1.pdf

Planning/Simulation:
1. Create package for source code
.Pseudo code:
1. Initialise node
2. Get Planning scene.(Layout of surrounding of ur5 from URDF)
3. Add Object constraints
4. Sucscribe to target location
5. Calculate trajectory
6. Publish to rviz

Move the arm:
1. Once we have successfully calculated the path we send trajectory to arm

Reference:
1. https://smartech.gatech.edu/bitstream/handle/1853/50782/ur_kin_tech_report_1.pdf
2. http://wiki.ros.org/Industrial/
3. http://wiki.ros.org/ROS/
4. http://moveit.ros.org/
5. http://wiki.ros.org/universal_robot/