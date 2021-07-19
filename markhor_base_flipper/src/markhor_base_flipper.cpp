#include <ros/ros.h>
#include "markhor_hw_interface_flipper.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "markhor_base_flipper_node");

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

    hw.read();
    cm.update(time, period);
    hw.write();

    rate.sleep();
  }

  return 0;
}
