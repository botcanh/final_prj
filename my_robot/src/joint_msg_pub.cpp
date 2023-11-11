#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>

int right_ticks = 0;
int left_ticks = 0;

const double PI = 3.141592;
const int TICKS_PER_REVOLUTION = 535; 

void rightTicksCallback(const std_msgs::Int16::ConstPtr& msg) {
    right_ticks = (msg->data + 32768) % TICKS_PER_REVOLUTION; //INT16 range
}

void leftTicksCallback(const std_msgs::Int16::ConstPtr& msg) {
    left_ticks = (msg->data + 32768) % TICKS_PER_REVOLUTION;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "encoder_to_joint_state_node");
    ros::NodeHandle nh;

    ros::Subscriber right_ticks_sub = nh.subscribe("/right_ticks", 1, rightTicksCallback);
    ros::Subscriber left_ticks_sub = nh.subscribe("/left_ticks", 1, leftTicksCallback);

    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

    ros::Rate rate(10); // 10 Hz, adjust as needed

    while (ros::ok()) {
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.stamp = ros::Time::now();
        joint_state_msg.name.push_back("base_to_lwheel");
        joint_state_msg.name.push_back("base_to_rwheel");

        // Calculate joint angles or positions based on encoder values

        // Fill in with calculated values
        double left_joint_position = ((double)left_ticks / TICKS_PER_REVOLUTION) * PI; // Calculate left joint position
        double right_joint_position = ((double)right_ticks / TICKS_PER_REVOLUTION) * PI; // Calculate right joint position

        joint_state_msg.position.push_back(left_joint_position);
        joint_state_msg.position.push_back(right_joint_position);

        joint_state_pub.publish(joint_state_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
