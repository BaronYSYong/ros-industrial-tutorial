# Exercise 3.2: Coordinate Transforms using TF

* Specify tf as a dependency of your core package.
    * Edit package.xml and CMakeLists.txt
    
> CMakeLists.txt
```
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    fake_ar_publisher 
    geometry_msgs 
    message_generation 
    tf
)
---------------
catkin_package( 
    CATKIN_DEPENDS 
    roscpp 
    fake_ar_publisher 
    message_runtime 
    geometry_msgs 
    tf
)
```
> package.xml
```
  <build_depend>tf</build_depend>
  <run_depend>tf</run_depend>
```
* Add a tf::TransformListener object to the vision node (as a class member variable).
```
#include <tf/transform_listener.h>
...
tf::TransformListener listener_;
```
* Add code to the existing localizePart method to convert the reported target pose from its reference frame ("camera_frame") to the service-request frame:
* For better or worse, ROS uses lots of different math libraries. You’ll need to transform the over-the-wire format of geometry_msgs::Pose into a tf::Transform object:
```c++
tf::Transform cam_to_target;
tf::poseMsgToTF(p->pose.pose, cam_to_target);
```
* Use the listener object to lookup the latest transform between the request.base_frame and the reference frame from the ARMarker message (which should be "camera_frame"):
``` c++
tf::StampedTransform req_to_cam;
listener_.lookupTransform(req.base_frame, p->header.frame_id, ros::Time(0), req_to_cam);
```
* Using the above information, transform the object pose into the target frame.
``` c++
tf::Transform req_to_target;
req_to_target = req_to_cam * cam_to_target;
```
* Return the transformed pose in the service response.
``` c++
tf::poseTFToMsg(req_to_target, res.pose);
```
* Run the nodes to test the transforms:
```
$ catkin_make
$ roslaunch myworkcell_support urdf.launch
$ roslaunch myworkcell_support workcell.launch
```
* Change the "base_frame" parameter in workcell.launch (e.g. to "table"), relaunch the workcell.launch file, and note the different pose result. Change the "base_frame" parameter back to "world" when you're done.