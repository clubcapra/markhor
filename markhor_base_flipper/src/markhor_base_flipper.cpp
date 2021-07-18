#include <ros/ros.h>
#include "markhor_hw_interface_flipper.hpp"

// void controlLoop(MarkhorHWInterfaceFlipper& hw, controller_manager::ControllerManager& cm,
//                  std::chrono::system_clock::time_point& last_time)
// {
//   std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
//   std::chrono::duration<double> elapsed_time = current_time - last_time;
//   ros::Duration elapsed(elapsed_time.count());
//   last_time = current_time;

//   hw.read(elapsed);
//   cm.update(ros::Time::now(), elapsed);
//   hw.write();
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "markhor_base_flipper_node");

  MarkhorHWInterfaceFlipper hw;
  controller_manager::ControllerManager cm(&hw);

  // double control_frequency;
  // hw.private_nh.param<double>("control_frequency", control_frequency, 10.0);

  // ros::CallbackQueue my_robot_queue;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(10.0);
  // std::chrono::system_clock::time_point last_time = std::chrono::system_clock::now();
  // ros::TimerOptions control_timer(ros::Duration(1 / control_frequency),
  //                                 std::bind(controlLoop, std::ref(hw), std::ref(cm), std::ref(last_time)),
  //                                 &my_robot_queue);
  // ros::Timer control_loop = hw.nh.createTimer(control_timer);
  // my_robot_spinner.start();
  // ros::spin();
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
