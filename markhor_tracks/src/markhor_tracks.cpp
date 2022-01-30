#include <chrono>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "markhor_hw_interface.hpp"

void controlLoop(MarkhorHWInterface& hw, controller_manager::ControllerManager& cm,
                 std::chrono::system_clock::time_point& last_time)
{
  std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_time = current_time - last_time;
  ros::Duration elapsed(elapsed_time.count());
  last_time = current_time;

  hw.read(elapsed);
  cm.update(ros::Time::now(), elapsed);
  hw.write();
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "markhor_tracks_node");

  ROS_INFO("LUDO %d",__LINE__);
  MarkhorHWInterface hw;
    ROS_INFO("LUDO %d",__LINE__);
  controller_manager::ControllerManager cm(&hw, hw.nh);

  ROS_INFO("LUDO %d",__LINE__);
  double control_frequency;
  ROS_INFO("LUDO %d",__LINE__);
  hw.private_nh.param<double>("control_frequency", control_frequency, 60.0);

  ROS_INFO("LUDO %d",__LINE__);
  ros::CallbackQueue my_robot_queue;
  ros::AsyncSpinner my_robot_spinner(1, &my_robot_queue);

  ROS_INFO("LUDO %d",__LINE__);
  std::chrono::system_clock::time_point last_time = std::chrono::system_clock::now();
  ROS_INFO("LUDO %d",__LINE__);
  ros::TimerOptions control_timer(ros::Duration(1 / control_frequency),
                                  std::bind(controlLoop, std::ref(hw), std::ref(cm), std::ref(last_time)),
                                  &my_robot_queue);
  ROS_INFO("LUDO %d",__LINE__);
  ros::Timer control_loop = hw.nh.createTimer(control_timer);
  ROS_INFO("LUDO %d",__LINE__);
  my_robot_spinner.start();
  ROS_INFO("LUDO %d",__LINE__);
  ros::spin();

  return 0;
}
