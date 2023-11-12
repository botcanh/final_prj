#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "initial_pose_publisher");
    ros::NodeHandle nh;

    ros::Publisher initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);

    ros::Rate loop_rate(10); // Publish at 10 Hz
    ros::Time start_time = ros::Time::now();
    ros::Duration publish_duration(10.0); // Publish for 10 seconds

    while (ros::ok() && (ros::Time::now() - start_time) < publish_duration) {
        // Create and populate the initial pose message
        geometry_msgs::PoseWithCovarianceStamped initial_pose_msg;
        initial_pose_msg.header.stamp = ros::Time::now();
        // ... (rest of the code to populate the message)
        initial_pose_msg.header.frame_id = "map"; // Set the appropriate frame_id
        initial_pose_msg.pose.pose.position.x = 1.0; // Set the initial x position
        initial_pose_msg.pose.pose.position.y = 2.0; // Set the initial y position
        initial_pose_msg.pose.pose.position.z = 0.0; // Z position
        initial_pose_msg.pose.pose.orientation.x = 0.0; // Orientation
        initial_pose_msg.pose.pose.orientation.y = 0.0;
        initial_pose_msg.pose.pose.orientation.z = 0.0;
        initial_pose_msg.pose.pose.orientation.w = 1.0; // 1.0 means no rotation

        // Publish the initial pose message
        initial_pose_pub.publish(initial_pose_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Finished publishing the initial pose.");
    return 0;
}
