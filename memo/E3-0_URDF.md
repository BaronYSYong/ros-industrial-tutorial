# Exercise 3.0: URDF (Unified Robot Description Format )

* Add a urdf sub-folder to your application support package.
* Create a new workcell.urdf file inside the urdf/ folder and insert the following XML skeleton:
```
<?xml version="1.0" ?>
<robot name="myworkcell" xmlns:xacro="http://ros.org/wiki/xacro">
</robot>
```
* Add the world frame as a "virtual link" (no geometry).
```
<link name="world"/>
```
* Add the table frame, and be sure to specify both collision & visual geometry tags. See the box type in the XML specification.
```
  <link name="table">
    <visual>
      <geometry>
        <box size="0.6 0.6 0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.6 0.05"/>
      </geometry>
    </collision>
  </link>
```
* Add the camera_frame frame as another virtual link (no geometry).
```
<link name="camera_frame"/>
```
* Connect your links with a pair of fixed joints Use an rpy tag in the world_to_camera joint to set its orientation as described in the introduction.
```
  <joint name="world_to_table" type="fixed">
    <parent link="world"/>
    <child link="table"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
  </joint>

  <joint name="world_to_camera" type="fixed">
    <parent link="world"/>
    <child link="camera_frame"/> 
    <origin xyz="0 0 1.0" rpy="0 1.571 0"/>
  </joint>
```
in terminal, type:
```
$ roslaunch urdf_tutorial xacrodisplay.launch model:=/home/baron/catkin_ws/src/ros-industrial/myworkcell_support/urdf/workcell.urdf
```