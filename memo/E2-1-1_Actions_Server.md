# Exercise 2.1.1: Actions Server

### Create Action Messages
```
$ cd ~/catkin_ws/src
$ $ catkin_create_pkg actionlib_tutorials actionlib_msgs
$ cd actionlib_tutorials
$ mkdir action
$ cd action
$ touch Fibonacci.action
```
Before writing an action it is important to define the goal, result, and feedback messages. The action messages are generated automatically from the .action file. This file defines the type and format of the goal, result, and feedback topics for the action. Create actionlib_tutorials/action/Fibonacci.action in your favorite editor, and place the following inside it: 
```
#goal definition
int32 order
---
#result definition
int32[] sequence
---
#feedback
int32[] sequence
```
Edit the CMakeLists.txt:

* add the actionlib_msgs package to the find_package macro's argument 
```
find_package(catkin REQUIRED COMPONENTS actionlib_msgs)
```
* Note that CMake needs to find_package actionlib_msgs (message_generation does not need to be listed explicitly, it is referenced implicitly by actionlib_msgs). 
* use the add_action_files macro to declare the actions you want to be generated: 
```
add_action_files(
  DIRECTORY action
  FILES Fibonacci.action
)
```
* call the generate_messages macro, not forgetting the dependencies on actionlib_msgs and other message packages like std_msgs: 
```
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)
```
* add actionlib_msgs to catkin_package macro like this: 
```
catkin_package(CATKIN_DEPENDS actionlib_msgs)
```
*  catkin_package also specifies only CATKIN_DEPEND to actionlib_msgs. The transitive dependency on message_runtime is happening automatically. 

Now by following, automatically generate msg files of your action files, and also see the result. 
```
$ cd catkin_ws/
$ catkin_make
$ ls devel/share/actionlib_tutorials/msg/
FibonacciActionFeedback.msg  FibonacciFeedback.msg
FibonacciActionGoal.msg      FibonacciGoal.msg
FibonacciAction.msg          FibonacciResult.msg
FibonacciActionResult.msg
$ ls devel/include/actionlib_tutorials/
FibonacciActionFeedback.h  FibonacciFeedback.h
FibonacciActionGoal.h      FibonacciGoal.h
FibonacciAction.h          FibonacciResult.h
FibonacciActionResult.h
```
To manually generate the message files from this file, use the script genaction.py from the actionlib_msgs package. 

### Simple Server
First, create actionlib_tutorials/src/fibonacci_server.cpp. 

#### Code explanation
```
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
```
actionlib/server/simple_action_server.h is the action library used from implementing simple actions.
```
#include <actionlib_tutorials/FibonacciAction.h>
```
This includes the action message generated from the Fibonacci.action file show above. This is a header generated automatically from the FibonacciAction.msg file. 
```
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  actionlib_tutorials::FibonacciFeedback feedback_;
  actionlib_tutorials::FibonacciResult result_;
```
These are the protected variables of the action class. The node handle is constructed and passed into the action server during construction of the action. The action server is constructed in the constructor of the action and has been discussed below. The feedback and result messages are created for publishing in the action. 
```
  FibonacciAction(std::string name) :
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }
```
In the action constructor, an action server is created. The action server takes arguments of a node handle, name of the action, and optionally an executeCB. In this example the action server is created with the arguments for the executeCB. 

Now the executeCB function referenced in the constructor is created. The callback function is passed a pointer to the goal message. Note: This is a boost shared pointer, given by appending "ConstPtr" to the end of the goal message type. 
```
 ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);
```
Here the internals of the action are created. In this example ROS_INFO is being published to let the user know that the action is executing. 
```
    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
```
An important component of an action server is the ability to allow an action client to request that the current goal execution be cancelled. When a client requests that the current goal be preempted the action server should cancel the goal, perform necessary clean-up, and call the function setPreempted(), which signals that the action has been preempted by user request. Setting the rate at which the action server checks for preemption requests is left to the implementor of the server. 
```
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }
```
Here the Fibonacci sequence is put into the feedback variable and then published on the feedback channel provided by the action server. Then the action continues on looping and publishing feedback.
```
    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }
```
Once the action has finished computing the Fibonacci sequence the action notifies the action client that the action is complete by setting succeeded. 
```
int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");

  FibonacciAction fibonacci("fibonacci");
  ros::spin();

  return 0;
}
```
Finally the main function, creates the action and spins the node. The action will be running and waiting to receive goals.

### Compiling
Add the following lines to your CMakeLists.txt file: 
```
add_executable(fibonacci_server src/fibonacci_server.cpp)

target_link_libraries(
  fibonacci_server
  ${catkin_LIBRARIES}
)

add_dependencies(
  fibonacci_server
  ${actionlib_tutorials_EXPORTED_TARGETS}
)
```

### Running Action server
Start a roscore in a new terminal 
```
$ roscore
```
And then run the action server: 
```
$ rosrun actionlib_tutorials fibonacci_server
```
To check that your action is running properly list topics being published: 
```
$ rostopic list -v

Published topics:
 * /fibonacci/feedback [actionlib_tutorials/FibonacciActionFeedback] 1 publisher
 * /fibonacci/status [actionlib_msgs/GoalStatusArray] 1 publisher
 * /rosout [rosgraph_msgs/Log] 1 publisher
 * /fibonacci/result [actionlib_tutorials/FibonacciActionResult] 1 publisher
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher

Subscribed topics:
 * /fibonacci/goal [actionlib_tutorials/FibonacciActionGoal] 1 subscriber
 * /fibonacci/cancel [actionlib_msgs/GoalID] 1 subscriber
 * /rosout [rosgraph_msgs/Log] 1 subscriber

```
