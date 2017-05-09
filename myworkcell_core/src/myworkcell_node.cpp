/// Services Client

#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group.h>

class ScanNPlan
{
public:
    ScanNPlan(ros::NodeHandle& nh)
    {
        vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
    }

    void start(const std::string& base_frame)
    {
        ROS_INFO("Attempting to localize part");
        // Localize the part
        myworkcell_core::LocalizePart srv;
        /// define srv.request.base_frame beforehand
        srv.request.base_frame = base_frame;
        ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);
        
        if (!vision_client_.call(srv))
        {
            ROS_ERROR("Could not localize part");
            return;
        }
        ROS_INFO_STREAM("part localized: " << srv.response);
        geometry_msgs::Pose move_target = flipPose(srv.response.pose);
        moveit::planning_interface::MoveGroup move_group("manipulator");
        // Plan for robot to move to part
        move_group.setPoseTarget(move_target); 
        move_group.move();
    }

    geometry_msgs::Pose flipPose(const geometry_msgs::Pose& in) const
    {
      tf::Transform in_tf; 
      tf::poseMsgToTF(in, in_tf);
      tf::Quaternion flip_rot(tf::Vector3(1, 0, 0), M_PI);
      tf::Transform flipped = in_tf * tf::Transform(flip_rot); 
      geometry_msgs::Pose out; 
      tf::poseTFToMsg(flipped, out);
      return out;
    }

private:
    // Planning components>
    ros::ServiceClient vision_client_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "myworkcell_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_node_handle ("~");
    std::string base_frame;

    /// parameter name, string object reference, default value
    private_node_handle.param<std::string>("base_frame", base_frame, "world"); 
    ros::AsyncSpinner async_spinner(1);
    
    ROS_INFO("ScanNPlan node has been initialized");

    ScanNPlan app(nh);

    ros::Duration(.5).sleep();  // wait for the class to initialize
    async_spinner.start();
    app.start(base_frame);

    //~ ros::spin();
    ros::waitForShutdown(); 
}
