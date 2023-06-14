#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <chrono>
#include <std_srvs/Trigger.h>

int ui_timeout;
int navigation_timeout;
int heartbeat_interval;
std::map<std::string, std::chrono::_V2::system_clock::time_point> last_heartbeats;

void heartbeat(const std_msgs::String message)
{
  last_heartbeats[message] = std::chrono::system_clock::now();
}

// Function that returns to known connection points until the heartbeat is back.
bool return_to_last_connection_point()
{
  // Call service
  return true;
}

bool validate_heartbeat(std::chrono::_V2::system_clock::time_point last_heartbeat, int timeout)
{
  std::chrono::_V2::system_clock::time_point now = std::chrono::system_clock::now();
  std::chrono::duration<double, std::milli> since_last_heartbeat = now - last_heartbeat;
  if (since_last_heartbeat.count() > timeout)
  {
    return false;
  }
  else
  {
    return true;
  }
}

void check_heartbeat()
{
  if (!validate_heartbeat(last_heartbeats.at("navigation"), navigation_timeout) &&
      !validate_heartbeat(last_heartbeats.at("ui"), ui_timeout))
  {
    bool success = return_to_connection_pose();
    std_srvs::Trigger service;
    if (ros::service::exists("/markhor/estop_disable", true))
    {
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
  nh.getParam("/markhor/heartbeat/markhor_heartbeat_node/navigation_timeout", navigation_timeout);
  nh.getParam("/markhor/heartbeat/markhor_heartbeat_node/ui_timeout", ui_timeout);

  ROS_INFO("Heartbeat interval is : %d ms", heartbeat_interval);
  ROS_INFO("Navigation timeout is : %d ms", navigation_timeout);
   ROS_INFO("UI timeout is : %d ms", ui_timeout);

  // Init heartbeats
  map["navigation"] = std::chrono::system_clock::now();
  map["ui"] = std::chrono::system_clock::now();

  // Subscribe to heartbeat topic to receive heartbeats from web ui.
  ros::Subscriber heartbeat_subscriber = nh.subscribe("heartbeat", 5, heartbeat);

  // Timer for UI heartbeat
  ros::Timer timer1 = nh.createTimer(ros::Duration(heartbeat_interval / 1000), check_heartbeat);

  ros::spin();
}