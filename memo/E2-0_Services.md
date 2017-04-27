# Exercise 2.0: Services

## Create Service Definition
Similar to the message file located in the fake_ar_publisher package, we need to create a service file. The following is a generic structure of a service file:
```
 #request
 ---
 #response
```

Create a folder called srv inside your myworkcell_core package (at same level as the package's src folder)
```
$ roscd myworkcell_core
$ mkdir srv
$ cd srv
$ touch LocalizePart.srv
```
Inside the file, define the service as outlined above with a request of type ```string``` named ```base_frame``` and a response of type ```geometry_msgs/Pose``` named ```pose```:
```
#request
string base_frame
---
#response
geometry_msgs/Pose pose
```
Edit the package's ```CMakeLists.txt``` and ```package.xml``` to add dependencies on key packages:

* ```message_generation``` is required to build C++ code from the .srv file created in the previous step
* ```message_runtime``` provides runtime dependencies for new messages
* ```geometry_msgs``` provides the ```Pose``` message type used in our service definition

Edit the package's CMakeLists.txt file to add the new build-time dependencies to the existing find_package
```
find_package(catkin REQUIRED COMPONENTS roscpp fake_ar_publisher geometry_msgs message_generation )
```
Also in CMakeLists.txt, add the new run-time dependencies to the existing catkin_package
```
catkin_package( 
    CATKIN_DEPENDS 
	roscpp 
	fake_ar_publisher 
	message_runtime 
	geometry_msgs 
)
```
Reference the LocalizePart service defined earlier:
```
 ## Generate services in the 'srv' folder
add_service_files(
    FILES
    LocalizePart.srv
)
 
  ## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    geometry_msgs
)
```
generate_messages() must be called before catkin_package() in project

Edit the package.xml file to add the appropriate build/run dependencies: 
```
<build_depend>message_generation</build_depend>
<run_depend>message_runtime</run_depend>
<build_depend>geometry_msgs</build_depend>
<run_depend>geometry_msgs</run_depend>
```
catkin_make the file

## Service Server
Edit ```myworkcell_core/src/vision_node.cpp```.   
Add the header for the service just created
```
#include <myworkcell_core/LocalizePart.h>
```
Add a member variable (type: ServiceServer, name: server_), near the other Localizer class member variables:
```
ros::ServiceServer server_;
```
In the Localizer class constructor, advertise your service to the ROS master:
```
server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this);
```
The ```advertiseService``` command above referenced a service callback named ```localizePart```. Create an empty boolean function with this name in the ```Localizer class```. Remember that your request and response types were defined in the ```LocalizePart.srv``` file. The arguments to the boolean function are the request and response type, with the general structure of ```Package::ServiceName::Request``` or ```Package::ServiceName::Response```.
```
bool localizePart(myworkcell_core::LocalizePart::Request& req, myworkcell_core::LocalizePart::Response& res)
 {

 }
```

Now add code to the ```localizePart``` callback function to fill in the Service Response. Eventually, this callback will transform the pose received from the ```fake_ar_publisher``` (in ```visionCallback```) into the frame specifed in the Service Request. For now, we will skip the frame-transform, and just pass through the data received from ```fake_ar_publisher```. Copy the pose measurement received from ```fake_ar_publisher``` (saved to ```last_msg_```) directly to the Service Response.
```
 bool localizePart(myworkcell_core::LocalizePart::Request& req, myworkcell_core::LocalizePart::Response& res)
 {
   // Read last message
   fake_ar_publisher::ARMarkerConstPtr p = last_msg_;  
   if (!p) return false;

   res.pose = p->pose.pose;
   return true;
 }
```
Comment out the ```ROS_INFO_STREAM``` call in your ```visionCallback``` function, to avoid cluttering the screen with useless info.

catkin_make the updated vision_node

## Service Client
Create a new node (inside the same myworkcell_core package), named myworkcell_node.cpp. This will eventually be our main "application node", that controls the sequence of actions in our Scan & Plan task. The first action we'll implement is to request the position of the AR target from the Vision Node's LocalizePart service we created above.

Be sure to include the standard ros header as well as the header for the LocalizePart service:
```
#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>
```
Create a standard C++ main function, with typical ROS node initialization:
```
int main(int argc, char **argv)
{
  ros::init(argc, argv, "myworkcell_node");
  ros::NodeHandle nh;

  ROS_INFO("ScanNPlan node has been initialized");

  ros::spin();
}
```
As in the vision_node, we will be using a cpp class "ScanNPlan" to contain most functionality of this node. Create a skeleton structure of this class, with an empty constructor and a private area for some internal/private variables.
```
class ScanNPlan
{
public:
  ScanNPlan(ros::NodeHandle& nh)
  {

  }

private:
  // Planning components

};
```
Within your new ScanNPlan class, define a ROS ServiceClient as a private member variable of the class. Initialize the ServiceClient in the ScanNPlan constructor, using the same service name as defined earlier ("localize_part"). Create a void function within the ScanNPlan class named start, with no arguments. This will contain most of our application workflow. For now, this function will call the LocalizePart service and print the response.
```
class ScanNPlan
{
public:
  ScanNPlan(ros::NodeHandle& nh)
  {
    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
  }

  void start()
  {
    ROS_INFO("Attempting to localize part");
    // Localize the part
    myworkcell_core::LocalizePart srv;
    if (!vision_client_.call(srv))
    {
      ROS_ERROR("Could not localize part");
      return;
    }
    ROS_INFO_STREAM("part localized: " << srv.response);
  }

private:
  // Planning components>
  ros::ServiceClient vision_client_;
};
```
Now back in myworkcell_node's main function, instantiate an object of the ScanNPlan class and call the object's start function.
```
ScanNPlan app(nh);

ros::Duration(.5).sleep();  // wait for the class to initialize
app.start();
```
Edit the package's CMakeLists.txt to build the new node (executable), with its associated dependencies. Add the following rules to the appropriate sections, directly under the matching rules for vision_node:
```
add_executable(myworkcell_node src/myworkcell_node.cpp)
...
add_dependencies(myworkcell_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
...
target_link_libraries(myworkcell_node ${catkin_LIBRARIES})
```
catkin_make the nodes

## Use New Service
```
$ roscore
$ rosrun fake_ar_publisher fake_ar_publisher_node
$ rosrun myworkcell_core vision_node
$ rosrun myworkcell_core myworkcell_node
```