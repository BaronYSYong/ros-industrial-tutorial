# Exerciese 1.3. Creating a ROS Package and Node

## Create a Package
```
$ cd ~/catkin_ws/src
$ catkin_create_pkg myworkcell_core roscpp
$ cd myworkcell_core
$ vim package.xml
```

## Create a node
```
$ vim CMakeLists.txt
```
```
add_definitions(-std=c++11)
add_executable(vision_node src/vision_node.cpp)
target_link_libraries(vision_node ${catkin_LIBRARIES})
```
```
$ cd src
$ touch vision_node.cpp
```
```
/**
**  Simple ROS Node
**/
#include <ros/ros.h>

int main(int argc, char* argv[])
{
       // This must be called before anything else ROS-related
       ros::init(argc, argv, "vision_node");

       // Create a ROS node handle
       ros::NodeHandle nh;

       ROS_INFO("Hello, World!");

       // Don't exit the program.
       ros::spin();
}
```
ROS_INFO is one of the many logging methods.

* It will print the message to the terminal output, and send it to the /rosout topic for other nodes to monitor.
* There are 5 levels of logging: DEBUG, INFO, WARNING, ERROR, & FATAL.
* To use a different logging level, replace INFO in ROS_INFO or ROS_INFO_STREAM with the appropriate level.
* To use printf-style logging, use ROS_INFO.

```
$ cd ~/catkin_ws
$ catkin_make
```

## Run a Node
Open a terminal and start the ROS master.
```
$ roscore
```
Open a second terminal to run your node
```
$ rosrun myworkcell_core vision_node
```
In a third terminal, check what nodes are running
```
$ rosnode list
```
In addition to the /rosout node, you should now see a new /vision_node listed. Enter rosnode kill /vision_node. This will stop the node.
```
$ rosnode kill /vision_node 
```
Note: It is more common to use Ctrl+C to stop a running node in the current terminal window.