#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>

#include "markhor_hw_interface_flippers.hpp"

static std_msgs::Float64 msg;

static ros::Publisher flipper_fl_pub;
static ros::Publisher flipper_fr_pub;
static ros::Publisher flipper_rl_pub;
static ros::Publisher flipper_rr_pub;

static bool flipper_mode_front = false;
static bool flipper_mode_back = false;

static int multiplicator = 0;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std_msgs::Float64 msg;
  if (flipper_mode_front == true)
  {
    msg.data = joy->axes[4] * multiplicator;
    flipper_fl_pub.publish(msg);
    flipper_fr_pub.publish(msg);
  }
  if (flipper_mode_back == true)
  {
    msg.data = joy->axes[4] * multiplicator;
    flipper_rl_pub.publish(msg);
    flipper_rr_pub.publish(msg);
  }
}

bool flipperModeFrontEnable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  flipper_mode_front = true;
  res.message = "successfully enable flipper mode front";
  res.success = static_cast<unsigned char>(true);
  return true;
}

bool flipperModeFrontDisable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  flipper_mode_front = false;
  res.message = "successfully disable flipper mode front";
  res.success = static_cast<unsigned char>(true);
  return true;
}

bool flipperModeBackEnable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  flipper_mode_back = true;
  res.message = "successfully enable flipper mode back";
  res.success = static_cast<unsigned char>(true);
  return true;
}

bool flipperModeBackDisable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  flipper_mode_back = false;
  res.message = "successfully disable flipper mode back";
  res.success = static_cast<unsigned char>(true);
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "markhor_flippers_node");
  ros::NodeHandle nh;

  if(nh.getParam("/markhor/markhor_flippers_node/multiplicator", multiplicator) == false)
  {
    ROS_ERROR("Missing multiplicator value from launch files."); 
    ros::shutdown();
    return 1;
  }

  flipper_fl_pub = nh.advertise<std_msgs::Float64>("flipper_fl_position_controller/command", 1000);
  flipper_fr_pub = nh.advertise<std_msgs::Float64>("flipper_fr_position_controller/command", 1000);
  flipper_rl_pub = nh.advertise<std_msgs::Float64>("flipper_rl_position_controller/command", 1000);
  flipper_rr_pub = nh.advertise<std_msgs::Float64>("flipper_rr_position_controller/command", 1000);

  ros::Subscriber joy_sub = nh.subscribe("/joy", 1000, joyCallback);

  ros::ServiceServer flipper_mode_front_enable = nh.advertiseService("flipper_mode_front_enable", flipperModeFrontEnable);
  ros::ServiceServer flipper_mode_front_disable = nh.advertiseService("flipper_mode_front_disable", flipperModeFrontDisable);
  ros::ServiceServer flipper_mode_back_enable = nh.advertiseService("flipper_mode_back_enable", flipperModeBackEnable);
  ros::ServiceServer flipper_mode_back_disable = nh.advertiseService("flipper_mode_back_disable", flipperModeBackDisable);

  MarkhorHWInterfaceFlippers hw;
  controller_manager::ControllerManager cm(&hw);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(10.0);

  while (ros::ok())
  {
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;

    hw.read();
    cm.update(time, period);
    hw.write();

    rate.sleep();
  }

  return 0;
}
