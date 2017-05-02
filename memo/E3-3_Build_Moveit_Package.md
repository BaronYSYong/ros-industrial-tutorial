# Exercise 3.3: Build a Moveit! Package

## MoveIt Setup

* Start the MoveIt! Setup Assistant
```
$ roslaunch moveit_setup_assistant setup_assistant.launch
```
* Select "Create New MoveIt Configuration Package", select the workcell.xacro you created previously, then "Load File".
* Generate a self-collision matrix.
![](/home/baron/catkin_ws/src/ros-industrial-tutorial/memo/image/self-collision.png) 
* Add a fixed virtual base joint.
```
name = 'FixedBase' (arbitrary)
child = 'world' (should match the URDF root link)
parent = 'world' (reference frame used for motion planning)
type = 'fixed'
```
![](/home/baron/catkin_ws/src/ros-industrial-tutorial/memo/image/virtual_joints.png) 

* Add a planning group called manipulator that names the kinematic chain between base_link and tool0
* Set the kinematics solver to `KDLKinematicsPlugin`
![](/home/baron/catkin_ws/src/ros-industrial-tutorial/memo/image/planning_groups.png) 

* Enter author / maintainer info.
![](/home/baron/catkin_ws/src/ros-industrial-tutorial/memo/image/author_info.png) 

* Create a package and name it myworkcell_moveit_config
* The outcome of these steps will be a new package that contains a large number of launch and configuration files. At this point, it's possible to do motion planning, but not to execute the plan on any robot. 
![](/home/baron/catkin_ws/src/ros-industrial-tutorial/memo/image/generate_config_file.png) 
```
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch myworkcell_moveit_config demo.launch
```
![](/home/baron/catkin_ws/src/ros-industrial-tutorial/memo/image/moveit_demo.png) 

## Using MoveIt!
* Create a controllers.yaml file (myworkcell_moveit_config/config/controllers.yaml) with the following contents:
```
controller_list:
  - name: ""
    action_ns: joint_trajectory_action
    type: FollowJointTrajectory
    joints: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
```
* Create the `joint_names.yaml` file (`myworkcell_moveit_config/config/joint_names.yaml`):
```
controller_joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
```
* Fill in the existing, but blank, controller_manager launch file (`myworkcell_moveit_config/launch/myworkcell_moveit_controller_manager.launch.xml`):
```
<launch>
  <arg name="moveit_controller_manager"
       default="moveit_simple_controller_manager/MoveItSimpleControllerManager"/>
  <param name="moveit_controller_manager"
         value="$(arg moveit_controller_manager)"/>

  <rosparam file="$(find myworkcell_moveit_config)/config/controllers.yaml"/>
</launch>
```
* Create a new myworkcell_planning_execution.launch (in myworkcell_moveit_config/launch):
```
<launch>
  <!-- The planning and execution components of MoveIt! configured to run -->
  <!-- using the ROS-Industrial interface. -->

  <!-- Non-standard joint names:
       - Create a file [robot_moveit_config]/config/joint_names.yaml
           controller_joint_names: [joint_1, joint_2, ... joint_N]
       - Update with joint names for your robot (in order expected by rbt controller)
       - and uncomment the following line: -->
  <rosparam command="load" file="$(find myworkcell_moveit_config)/config/joint_names.yaml"/>

  <!-- the "sim" argument controls whether we connect to a Simulated or Real robot -->
  <!--  - if sim=false, a robot_ip argument is required -->
  <arg name="sim" default="true" />
  <arg name="robot_ip" unless="$(arg sim)" />

  <!-- load the robot_description parameter before launching ROS-I nodes -->
  <include file="$(find myworkcell_moveit_config)/launch/planning_context.launch" >
    <arg name="load_robot_description" value="true" />
  </include>

  <!-- run the robot simulator and action interface nodes -->
  <group if="$(arg sim)">
    <include file="$(find industrial_robot_simulator)/launch/robot_interface_simulator.launch" />
  </group>

  <!-- run the "real robot" interface nodes -->
  <!--   - this typically includes: robot_state, motion_interface, and joint_trajectory_action nodes -->
  <!--   - replace these calls with appropriate robot-specific calls or launch files -->
  <group unless="$(arg sim)">
    <include file="$(find ur_bringup)/launch/ur5_bringup.launch" />
  </group>

  <!-- publish the robot state (tf transforms) -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

  <include file="$(find myworkcell_moveit_config)/launch/move_group.launch">
    <arg name="publish_monitored_planning_scene" value="true" />
  </include>

  <include file="$(find myworkcell_moveit_config)/launch/moveit_rviz.launch">
    <arg name="config" value="true"/>
  </include>

</launch>
```
* Add the source files to repository:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-industrial/industrial_core.git
$ catkin build    (this will take several minutes)
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch myworkcell_moveit_config myworkcell_planning_execution.launch
```
![](/home/baron/catkin_ws/src/ros-industrial-tutorial/memo/image/using_moveit.png) 