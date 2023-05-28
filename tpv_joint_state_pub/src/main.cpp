#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

    sensor_msgs::JointState msg;
    msg.name.push_back("tpv_base_j");
    msg.name.push_back("tpv_tilt_j");
    msg.position.push_back(0.0);  // Initial position of joint1
    msg.position.push_back(0.0);

    ros::Rate loop_rate(10);

    double time = 0.0;

    while (ros::ok())
    {
        msg.header.stamp = ros::Time::now();
        msg.position[0] = 3.14*std::sin(time*0.5);  // Update position of joint1
        msg.position[1] = 3.14*std::cos(time*0.5);
        
        state_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        time += 0.1;
    }

    return 0;
}