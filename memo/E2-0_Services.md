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