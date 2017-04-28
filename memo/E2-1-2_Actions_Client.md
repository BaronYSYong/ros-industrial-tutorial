# Exercise 2.1.2: Actions Client

## Simple Client
First, create actionlib_tutorials/src/fibonacci_client.cpp

### Code explanation
```
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
```

* actionlib/client/simple_action_client.h is the action library used from impleactionlib/client/terminal_state.h defines the possible goal states. menting simple action clients. 
* actionlib/client/terminal_state.h defines the possible goal states. 
```
#include <actionlib_tutorials/FibonacciAction.h>
```
This includes action message generated from the Fibonacci.action file shown above. This is a header generated automatically from the FibonacciAction.msg file.
```
int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);
```
The action client is templated on the action definition, specifying what message types to communicate to the action server with. The action client constructor also takes two arguments, the server name to connect to and a boolean option to automatically spin a thread. If you prefer not to use threads (and you want actionlib to do the 'thread magic' behind the scenes), this is a good option for you. Here the action client is constructed with the server name and the auto spin option set to true. 
```
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
```
Since the action server may not be up and running, the action client will wait for the action server to start before continuing. 
```
  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  actionlib_tutorials::FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal);
```
Here a goal message is created, the goal value is set and sent to the action server.
```
  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
```
The action client now waits for the goal to finish before continuing. The timeout on the wait is set to 30 seconds, this means after 30 seconds the function will return with false if the goal has not finished. 
```
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
```
If the goal finished before the time out the goal status is reported, else the user is notified that the goal did not finish in the allotted time. 

## Compiling
Add the following lines to CMakeLists.txt file: 
```
add_executable(fibonacci_client src/fibonacci_client.cpp)

target_link_libraries( 
  fibonacci_client
  ${catkin_LIBRARIES}
)

add_dependencies(
  fibonacci_client
  ${actionlib_tutorials_EXPORTED_TARGETS}
)
```
The build
```
cd %TOPDIR_YOUR_CATKIN_WORKSPACE%
catkin_make
source devel/setup.bash
```

## Running the Action Client
After you have successfully built the executable, start a new terminal and run the client: 
```
$ rosrun actionlib_tutorials fibonacci_client
```
To check that your action is running properly list topics being published: 
```
$ rostopic list -v

Published topics:
 * /fibonacci/goal [actionlib_tutorials/FibonacciActionGoal] 1 publisher
 * /fibonacci/cancel [actionlib_msgs/GoalID] 1 publisher
 * /rosout [rosgraph_msgs/Log] 1 publisher
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher

Subscribed topics:
 * /fibonacci/feedback [actionlib_tutorials/FibonacciActionFeedback] 1 subscriber
 * /fibonacci/status [actionlib_msgs/GoalStatusArray] 1 subscriber
 * /rosout [rosgraph_msgs/Log] 1 subscriber
 * /fibonacci/result [actionlib_tutorials/FibonacciActionResult] 1 subscriber
```