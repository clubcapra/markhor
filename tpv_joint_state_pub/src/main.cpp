#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "math.h"

#define X_REDUCTION 0.25
#define DEG_TO_RAD 180.0/M_PI

sensor_msgs::JointState joint_state_msg;

void callback_x(const std_msgs::Float64::ConstPtr& msg)
{
    joint_state_msg.position[0] = msg->data*DEG_TO_RAD*X_REDUCTION;
}

void callback_y(const std_msgs::Float64::ConstPtr& msg)
{
    joint_state_msg.position[1] = msg->data*DEG_TO_RAD;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;

    ros::Publisher state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.name.push_back("tpv_base_j");
    joint_state_msg.name.push_back("tpv_tilt_j");
    joint_state_msg.position.push_back(0.0); 
    joint_state_msg.position.push_back(0.0);

    ros::Subscriber sub_x = nh.subscribe("tpv_pos_x", 1, callback_x);
    ros::Subscriber sub_y = nh.subscribe("tpv_pos_y", 1, callback_y);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        joint_state_msg.header.stamp = ros::Time::now();

        state_pub.publish(joint_state_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}