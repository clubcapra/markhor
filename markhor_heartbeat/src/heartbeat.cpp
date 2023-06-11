#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <chrono>

bool lost_connection = false;
bool autonomy_mode = false;
int heartbeat_timeout;
int heartbeat_interval;
int reconnection_attempts = 0;
std::chrono::_V2::system_clock::time_point last_heartbeat = std::chrono::system_clock::now();

void heartbeat(const std_msgs::String message)
{
  last_heartbeat = std::chrono::system_clock::now();
}

// Function that returns to known connection points until the heartbeat is back.
bool return_to_connection_pose(int attempts = 0)
{
  MoveBaseClient ac("move_base", true);
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Get last connection point
  ros::NodeHandle nh;
  geometry_msgs::PoseStamped connection_pose;

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  ac.sendGoal(goal);

  // Wait for the action to finish
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Robot has reached last known connection point.");
    // Check again if we received a heartbeat after the robot has reached destination.
    if(validate_heartbeat()){
      lost_connection = false;
    }else{
      // If we still don't have a heartbeat, try again to go back to a later known connection point.
      return_to_connection_pose(++attempts);
    }
  }
  else
  {
    ROS_INFO("The robot failed to move to the last known connection point.");
    std_srvs::Trigger service;
      if (ros::service::exists("/markhor/estop_disable", true))
      {
        ros::service::call("/markhor/estop_disable", service);
      }
    return false;
  }
}

bool validate_heartbeat(){
  std::chrono::_V2::system_clock::time_point now = std::chrono::system_clock::now();
  std::chrono::duration<double, std::milli> since_last_heartbeat = now - last_heartbeat;
  if (since_last_heartbeat.count() > heartbeat_timeout)
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
  bool heartbeat_valid = validate_heartbeat();
  // If we lost the connection, bring back robot to last known connection point
  if (!autonomy_mode && !lost_connection && !heartbeat_valid)
  {
    ROS_WARN("Heartbeat timeout! Returning to last known connection point.");
    lost_connection = true;
    // Go back to known connection points until the heartbeat is back.
    return_to_connection_pose();
  }
  else if(lost_connection && since_last_heartbeat.count() < heartbeat_timeout){
    lost_connection = false;
  }
  
}

// Allow to toggle autonomy mode. If autonomy mode is enabled, the robot will not be stopped if the heartbeat is lost.
bool toggleAutonomyMode(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  autonomy_mode = req.data;
  if (autonomy_mode)
  {
    res.message = "Autonomy mode enabled.";
  }
  else
  {
    res.message = "Autonomy mode disabled.";
  }

  res.success = static_cast<unsigned char>(true);
  return true;
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
  ros::ServiceServer toggle_autonomy_mode = nh.advertiseService("/heartbeat/toggle_autonomy_mode", toggleAutonomyMode);

  // Create timer to verify that we received a heartbeat.
  ros::Timer timer1 = nh.createTimer(ros::Duration(heartbeat_interval / 1000), check_heartbeat);

  ros::spin();
}