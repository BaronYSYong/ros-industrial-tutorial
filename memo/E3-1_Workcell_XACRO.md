# Exercise 3.1: Workcell XACRO

* Rename the workcell.urdf file from the previous exercise to workcell.xacro
* Bring in the ur_description package into your ROS environment. You have a few options:
    * You can install the debian packages: sudo apt install ros-kinetic-universal-robot
    * You can clone it from GitHub to your catkin workspace:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-industrial/universal_robot.git
$ catkin build
$ source ~/catkin_ws/devel/setup.bash
```
>
It’s not uncommon for description packages to put each “module”, “part”, or “assembly” into its own file. In many cases, a package will also define extra files that define a complete cell with the given part so that we can easily visually inspect the result. The UR package defines such a file for the UR5 (ur5_robot.urdf.xacro): It’s a great example for this module.
https://github.com/ros-industrial/universal_robot/blob/indigo-devel/ur_description/urdf/ur5_robot.urdf.xacro

* Locate the xacro file that implements the UR5 macro and include it in your newly renamed workcell.xacro file. Add this include line near the top of your workcell.xacro file, beneath the <robot> tag:
```
<xacro:include filename="$(find ur_description)/urdf/ur5.urdf.xacro" />
```
>
If you explore the UR5 definition file, or just about any other file that defines a Xacro macro, you’ll find a lot of uses of ```${prefix}``` in element names. Xacro evaluates anything inside a ```“${}”``` at run-time. It can do basic math, and it can look up variables that come to it via properties (ala-global variables) or macro parameters. Most macros will take a “prefix” parameter to allow a user to create multiple instances of said macro. It’s the mechanism by which we can make the eventual URDF element names unique, otherwise we’d get duplicate link names and URDF would complain.

* Including the ur5.urdf.xacro file does not actually create a UR5 robot in our URDF model. It defines a macro, but we still need to call the macro to create the robot links and joints. Note the use of the prefix tag, as discussed above.
```
<xacro:ur5_robot prefix="" joint_limited="true"/>
```
>
Macros in Xacro are just fancy wrappers around copy-paste. You make a macro and it gets turned into a chunk of links and joints. You still have to connect the rest of your world to that macro’s results. This means you have to look at the macro and see what the base link is and what the end link is. Hopefully your macro follows a standard, like the ROS-Industrial one, that says that base links are named “base_link” and the last link is called “tool0”.

* Connect the UR5 base_link to your existing static geometry with a fixed link.
```
<joint name="table_to_robot" type="fixed">
  <parent link="table"/>
  <child link="base_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```
* Create a new urdf.launch file (in the myworkcell_support package) to load the URDF model and (optionally) display it in rviz:
```
<launch>
  <arg name="gui" default="true"/>
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find myworkcell_support)/urdf/workcell.xacro'" />
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="$(arg gui)"/>
  </node>
  <node name="rviz" pkg="rviz" type="rviz" if="$(arg gui)"/>
</launch>
```
* Check the updated URDF in RViz, using the launch file you just created:
```$ roslaunch myworkcell_support urdf.launch```
    * Set the 'Fixed Frame' to 'world' and add the RobotModel and TF displays to the tree view on the left, to show the robot and some transforms.
    * Try moving the joint sliders to see the UR5 robot move.