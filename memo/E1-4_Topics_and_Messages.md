# Exercise 1.4: Topics and Messages

## Add the fake_ar_publisher Package as a Dependency
Locate the ```fake_ar_publisher``` package
```
$ rospack find fake_ar_publisher
```
Edit CMakeLists.txt file (~/catkin_ws/src/myworkcell_core/CMakeLists.txt). Tell cmake to find the fake_ar_publisher package:
```
 find_package(catkin REQUIRED COMPONENTS 
   roscpp 
   fake_ar_publisher
 )
```
Add The catkin runtime dependency for publisher.
```
 catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES lesson_simple_topic
    CATKIN_DEPENDS 
      roscpp 
      fake_ar_publisher
 #  DEPENDS system_lib
 )
```
Add ```add_dependencies``` line below ```add_executable```
```
add_dependencies(vision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```
Add dependencies into package's ```package.xml```:
```
 <build_depend>fake_ar_publisher</build_depend>
 <run_depend>fake_ar_publisher</run_depend>
```
Build your package and source the setup file to activate the changes in the current terminal
```
$ cd ~/catkin_ws
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```
In a terminal, enter ```rosmsg list```. You will notice that, included in the list, is fake_ar_publisher/ARMarker. If you want to see only the messages in a package, type rosmsg package <package_name>
```
$ rosmsg package fake_ar_publisher 
fake_ar_publisher/ARMarker
```
Type ```rosmsg show fake_ar_publisher/ARMarker```. The terminal will return the types and names of the fields in the message.
```
$ rosmsg show fake_ar_publisher/ARMarker
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
uint32 confidence
```
Note that three fields under the ```header``` field are indented, indicating that these are members of the ```std_msgs/Header``` message type

## Run Publisher Node
In a terminal, type 
```
$ rosrun fake_ar_publisher fake_ar_publisher_node
[ INFO] [1493187241.874826235]: Starting simulated ARMarker publisher
```

In another terminal, enter 
```
$ rostopic list
/ar_pose_marker
/ar_pose_visual
/rosout
/rosout_agg
```
You should see ```/ar_pose_marker``` among the topics listed. Entering ```rostopic type /ar_pose_marker``` will return the type of the message.

Enter ```rostopic echo /ar_pose_marker```. The terminal will show the fields for each message as they come in, separated by a ```---``` line. Press Ctrl+C to exit.

Enter ```rqt_plot```. 

Once the window opens, type ```/ar_pose_marker/pose/pose/position/x``` in the "Topic:" field and click the "+" button. You should see the X value be plotted.

Type ```/ar_pose_marker/pose/pose/position/y``` in the topic field, and click on the add button. You will now see both the x and y values being graphed.

## Create Subscriber Node
Edit the ```vision_node.cpp``` file. Include the message type as a header
```
#include <fake_ar_publisher/ARMarker.h>
```
Add the code that will be run when a message is received from the topic (the callback).
```
class Localizer
{
public:
    Localizer(ros::NodeHandle& nh)
    {
        ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1, 
        &Localizer::visionCallback, this);
    }

    void visionCallback(const fake_ar_publisher::ARMarkerConstPtr& msg)
    {
        last_msg_ = msg;
        ROS_INFO_STREAM(last_msg_->pose.pose);
    }
 
    ros::Subscriber ar_sub_;
    fake_ar_publisher::ARMarkerConstPtr last_msg_;
};
```
Add the code that will connect the callback to the topic (within ```main()```)
```
int main(int argc, char** argv)
{
    ...
    // The Localizer class provides this node's ROS interfaces
    Localizer localizer(nh);

    ROS_INFO("Vision node starting");
}
```
Run ```catkin_make```, then ```rosrun myworkcell_core vision_node```

You should see the positions display from the publisher.
Use ```rqt_graph``` to check the architecture
