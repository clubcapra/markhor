#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>

#include "markhor_hw_interface_flippers.hpp"

static ros::Publisher flipper_fl_pub;
static ros::Publisher flipper_fr_pub;
static ros::Publisher flipper_rl_pub;
static ros::Publisher flipper_rr_pub;

static bool flipper_mode_fl = false;
static bool flipper_mode_fr = false;
static bool flipper_mode_rl = false;
static bool flipper_mode_rr = false;

static int multiplicator = 0;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std_msgs::Float64 msg;

  if (flipper_mode_fl == true)
  {
    msg.data = joy->axes[4] * multiplicator;
    flipper_fl_pub.publish(msg);
  }
  if (flipper_mode_fr == true)
  {
    msg.data = joy->axes[4] * multiplicator;
    flipper_fr_pub.publish(msg);
  }
  if (flipper_mode_rl == true)
  {
    msg.data = joy->axes[4] * multiplicator;
    flipper_rl_pub.publish(msg);
  }
  if (flipper_mode_rr == true)
  {
    msg.data = joy->axes[4] * multiplicator;
    flipper_rr_pub.publish(msg);
  }
}

bool flipperModeFrontEnable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  flipper_mode_fl = true;
  flipper_mode_fr = true;
  res.message = "successfully enabled flipper mode front";
  res.success = static_cast<unsigned char>(true);
  return true;
}

bool flipperModeFrontDisable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  flipper_mode_fl = false;
  flipper_mode_fr = false;
  res.message = "successfully disabled flipper mode front";
  res.success = static_cast<unsigned char>(true);
  return true;
}

bool flipperModeRearEnable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  flipper_mode_rl = true;
  flipper_mode_rr = true;
  res.message = "successfully enabled flipper mode rear";
  res.success = static_cast<unsigned char>(true);
  return true;
}

bool flipperModeRearDisable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  flipper_mode_rl = false;
  flipper_mode_rr = false;
  res.message = "successfully disabled flipper mode rear";
  res.success = static_cast<unsigned char>(true);
  return true;
}

bool flipperModeFLEnable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  flipper_mode_fl = true;
  res.message = "successfully enabled flipper mode front left";
  res.success = static_cast<unsigned char>(true);
  return true;
}

bool flipperModeFLDisable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  flipper_mode_fl = false;
  res.message = "successfully disabled flipper mode front left";
  res.success = static_cast<unsigned char>(true);
  return true;
}

bool flipperModeFREnable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  flipper_mode_fr = true;
  res.message = "successfully enabled flipper mode front right";
  res.success = static_cast<unsigned char>(true);
  return true;
}

bool flipperModeFRDisable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  flipper_mode_fr = false;
  res.message = "successfully disabled flipper mode front right";
  res.success = static_cast<unsigned char>(true);
  return true;
}

bool flipperModeRLEnable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  flipper_mode_rl = true;
  res.message = "successfully enabled flipper mode rear left";
  res.success = static_cast<unsigned char>(true);
  return true;
}

bool flipperModeRLDisable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  flipper_mode_rl = false;
  res.message = "successfully disabled flipper mode rear left";
  res.success = static_cast<unsigned char>(true);
  return true;
}

bool flipperModeRREnable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  flipper_mode_rr = true;
  res.message = "successfully enabled flipper mode rear right";
  res.success = static_cast<unsigned char>(true);
  return true;
}

bool flipperModeRRDisable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  flipper_mode_rr = false;
  res.message = "successfully disabled flipper mode rear right";
  res.success = static_cast<unsigned char>(true);
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "markhor_flippers_node");
  ros::NodeHandle nh;

  if (nh.getParam("/markhor/flippers/markhor_flippers_node/multiplicator", multiplicator) == false)
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

  ros::ServiceServer flipper_mode_front_enable =
      nh.advertiseService("flipper_mode_front_enable", flipperModeFrontEnable);
  ros::ServiceServer flipper_mode_front_disable =
      nh.advertiseService("flipper_mode_front_disable", flipperModeFrontDisable);
  ros::ServiceServer flipper_mode_rear_enable = nh.advertiseService("flipper_mode_rear_enable", flipperModeRearEnable);
  ros::ServiceServer flipper_mode_rear_disable =
      nh.advertiseService("flipper_mode_rear_disable", flipperModeRearDisable);

  ros::ServiceServer flipper_mode_fl_enable = nh.advertiseService("flipper_mode_fl_enable", flipperModeFLEnable);
  ros::ServiceServer flipper_mode_fl_disable = nh.advertiseService("flipper_mode_fl_disable", flipperModeFLDisable);

  ros::ServiceServer flipper_mode_fr_enable = nh.advertiseService("flipper_mode_fr_enable", flipperModeFREnable);
  ros::ServiceServer flipper_mode_fr_disable = nh.advertiseService("flipper_mode_fr_disable", flipperModeFRDisable);

  ros::ServiceServer flipper_mode_rl_enable = nh.advertiseService("flipper_mode_rl_enable", flipperModeRLEnable);
  ros::ServiceServer flipper_mode_rl_disable = nh.advertiseService("flipper_mode_rl_disable", flipperModeRLDisable);

  ros::ServiceServer flipper_mode_rr_enable = nh.advertiseService("flipper_mode_rr_enable", flipperModeRREnable);
  ros::ServiceServer flipper_mode_rr_disable = nh.advertiseService("flipper_mode_rr_disable", flipperModeRRDisable);

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
    std_msgs::Float64 clear_queue_msg;
    clear_queue_msg.data = 0;

    if (flipper_mode_fl == false)
    {
      flipper_fl_pub.publish(clear_queue_msg);
    }

    if (flipper_mode_fr == false)
    {
      flipper_fr_pub.publish(clear_queue_msg);
    }

    if (flipper_mode_rl == false)
    {
      flipper_rl_pub.publish(clear_queue_msg);
    }

    if (flipper_mode_rr == false)
    {
      flipper_rr_pub.publish(clear_queue_msg);
    }

    hw.read();
    cm.update(time, period);
    hw.write();

    rate.sleep();
  }

  return 0;
}