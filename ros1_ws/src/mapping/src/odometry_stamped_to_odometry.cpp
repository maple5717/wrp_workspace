#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>  // Replace with the actual package name

class OdometryConverter
{
public:
    OdometryConverter()
    {
        // Initialize the node handle
        ros::NodeHandle nh;

        // Subscribe to OdometryStamped topic
        sub_ = nh.subscribe("/slam_out_pose", 10, &OdometryConverter::callback, this);

        // Publish Odometry messages
        pub_ = nh.advertise<nav_msgs::Odometry>("/hector_out_pose", 10);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;

    void callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        // Convert OdometryStamped to Odometry
        nav_msgs::Odometry odometry_msg;

        odometry_msg.header = msg->header;
        odometry_msg.child_frame_id = "base_link";
        odometry_msg.pose.pose = msg->pose;
        // odometry_msg.twist = msg->msg.twist;

        // Publish the converted Odometry message
        pub_.publish(odometry_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_stamped_to_odometry");

    OdometryConverter converter;

    ros::spin();

    return 0;
}
