#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "send_vel");
    ros::NodeHandle nh;
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        geometry_msgs::Twist msg;
        msg.linear.x = 0.5;
        msg.linear.y = 0;
        msg.linear.z = 0;

        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;

        cmd_pub.publish(msg);
        ROS_INFO("%f", msg.angular.z);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}