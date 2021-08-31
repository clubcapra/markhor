#include <ros/ros.h>
#include "markhor_hw_interface_flipper.hpp"
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>

static std_msgs::Float64 msg;
static float accumulator_rl = 0;
static ros::Publisher flipper_fl_pub;
static ros::Publisher flipper_fr_pub;
static ros::Publisher flipper_rl_pub;
static ros::Publisher flipper_rr_pub;
static float accumulator = 0;
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std_msgs::Float64 msg;
  if (joy->buttons[0] == 1)
  {
    msg.data = joy->axes[1] * 1500;  // TODO : Set the multiplicator inside the launch file
    flipper_rl_pub.publish(msg);
    flipper_rr_pub.publish(msg);
  }
  else
  {
    msg.data = joy->axes[1] * 0;
    flipper_rl_pub.publish(msg);
    flipper_rr_pub.publish(msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "markhor_base_flipper_node");
  ros::NodeHandle nh;

  flipper_fl_pub = nh.advertise<std_msgs::Float64>("flipper_fl_position_controller/command", 1000);
  flipper_fr_pub = nh.advertise<std_msgs::Float64>("flipper_fr_position_controller/command", 1000);
  flipper_rl_pub = nh.advertise<std_msgs::Float64>("flipper_rl_position_controller/command", 1000);
  flipper_rr_pub = nh.advertise<std_msgs::Float64>("flipper_rr_position_controller/command", 1000);

  ros::Subscriber joy_sub = nh.subscribe("/joy", 1000, joyCallback);

  MarkhorHWInterfaceFlipper hw;
  controller_manager::ControllerManager cm(&hw);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(10.0);

  while (ros::ok())
  {
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;

    // flipper_fl_pub.publish(msg);

    hw.read();
    cm.update(time, period);
    hw.write();

    rate.sleep();
  }

  return 0;
}
