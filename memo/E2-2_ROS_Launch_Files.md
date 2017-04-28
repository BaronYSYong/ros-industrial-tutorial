# Exercise 2.2: ROS Launch Files

Create the new package myworkcell_support with a dependency on myworkcell_core. Rebuild and source the workspace so that ROS can find the new package
```
$ cd ~/catkin_ws/src/
$ catkin_create_pkg myworkcell_support myworkcell_core
$ cd ..
$ catking_make
$ source ~/catkin_ws/devel/setup.bash
```

Create a directory for launch files (inside the new myworkcell_support package):
```
$ roscd myworkcell_support
$ mkdir launch
$ cd launch
$ touch workcell.launch
```
Fill this into workcell.launch
```
<launch>
 <node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" />
 <node name="vision_node" pkg="myworkcell_core" type="vision_node" />
</launch>
```
Test the launch file:
```
$ roslaunch myworkcell_support workcell.launch
```
Notice that none of the usual messages were printed to the console window. Launch files will suppress console output below the ERROR severity level by default. To restore normal text output, add an extra tag to each of the nodes in your launch files:
```
<launch>
 <node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" output="screen"/>
 <node name="vision_node" pkg="myworkcell_core" type="vision_node" output="screen"/>
</launch>
```
