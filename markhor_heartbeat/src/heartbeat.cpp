#include <ros/ros.h>
#include "std_srvs/Trigger.h"
#include <std_msgs/String.h>
#include <chrono>

bool stopped = false;
int heartbeat_timeout;
int heartbeat_interval;
std::chrono::_V2::system_clock::time_point last_heartbeat = std::chrono::system_clock::now();

void heartbeat(const std_msgs::String message)
{
    last_heartbeat = std::chrono::system_clock::now();
}

void check_heartbeat(const ros::TimerEvent& e, ros::NodeHandle nh) {
    std::chrono::_V2::system_clock::time_point now = std::chrono::system_clock::now();
    std::chrono::duration<double, std::milli> since_last_heartbeat = now - last_heartbeat;

    if (stopped && since_last_heartbeat.count() < heartbeat_timeout) {
        ROS_WARN("Connection restored.");
        stopped = false;

        std_srvs::Trigger service;
        if (ros::service::exists("/markhor/estop_enable", true)) {
            ros::service::call("/markhor/estop_enable", service);
        }
    } else if (!stopped && since_last_heartbeat.count() > heartbeat_timeout) {
        ROS_WARN("Heartbeat timeout! Shutting down.");
        stopped = true;

        std_srvs::Trigger service;
        if (ros::service::exists("/markhor/estop_disable", true)) {
            ros::service::call("/markhor/estop_disable", service);
        }
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "markhor_heartbeat_node");
    ros::NodeHandle nh;

    // Get parameters from launch file.
    nh.getParam("/markhor/heartbeat/markhor_heartbeat_node/heartbeat_interval", heartbeat_interval);
    nh.getParam("/markhor/heartbeat/markhor_heartbeat_node/heartbeat_timeout", heartbeat_timeout);

    ROS_INFO("Heartbeat interval is : %d ms", heartbeat_interval);
    ROS_INFO("Heartbeat timeout is : %d ms", heartbeat_timeout);

    // Subscribe to heartbeat topic to receive heartbeats from web ui.
    ros::Subscriber heartbeat_subscriber = nh.subscribe("/heartbeat", 5, heartbeat);

    // Create timer to verify that we received a heartbeat.
    ros::Timer timer1 = nh.createTimer(ros::Duration(heartbeat_interval/1000), boost::bind(check_heartbeat, _1, nh));

    ros::spin();
}