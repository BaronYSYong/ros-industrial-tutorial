# Exercise 2.3: Parameters

* Open up myworkcell_node.cpp for editing.
* Add a new ros::NodeHandle object to the main function, and make it
```
ros::NodeHandle private_node_handle ("~");
```
* Create a temporary string object, std::string base_frame;, and then use the private node handle's API to load the parameter "base_frame".
```
std::string base_frame;
private_node_handle.param<std::string>("base_frame", base_frame, "world"); // parameter name, string object reference, default value
```
* Add a parameter to your myworkcell_node "start" function that accepts the base_frame argument, and assign the value from the parameter into the service request. Make sure to update the app.start call in your main() routine to pass through the base_frame value you read from the parameter server:
```
 void start(const std::string& base_frame)
 {
   ...
   srv.request.base_frame = base_frame;
   ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);
   ...
 }

 void main(...)
 {
   ...
   app.start(base_frame);
   ...
 }
```
* Now we'll add myworkcell_node to the existing workcell.launch file, so we can set the base_frame parameter from a launch file. We'd like the vision_node to return the position of the target relative to the world frame, for motion-planning purposes. Even though that's the default value, we'll specify it in the launch-file anyway:
```
 <node name="myworkcell_node" pkg="myworkcell_core" type="myworkcell_node" output="screen">
   <param name="base_frame" value="world"/>
 </node>
```
* Try it out by running the system.
```
$ catkin_make
$ roslaunch myworkcell_support workcell.launch
```
* Press _Ctrl+C_ to kill the running nodes
* Edit the launch file to change the base_frame parameter value (e.g. to "test2")
* Re-launch workcell.launch, and observe that the "request frame" has changed
     - The response frame doesn't change, because we haven't updated vision_node (yet) to handle the request frame.  Vision_node always returns the same frame (for now).
* Set the base_frame back to "world"