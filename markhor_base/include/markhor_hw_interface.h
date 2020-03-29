#ifndef CAPRA_CONTROL_H
#define CAPRA_CONTROL_H
// ROS
#include <ros/ros.h>
// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class MarkhorHWInterface : public hardware_interface::RobotHW
{
public:
  MarkhorHWInterface();

  void write()
  {
    ROS_INFO("TEST WRITE");
  }

  void read(const ros::Duration& period)
  {
    ROS_INFO("TEST WRITE");
  }

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd;
  double pos;
  double vel;
  double eff;
  ros::Time curr_update_time, prev_update_time;
};  // class

MarkhorHWInterface::MarkhorHWInterface() : private_nh("~")
{
  hardware_interface::JointStateHandle state_handle("Test_CC", &pos, &vel, &eff);
  jnt_state_interface.registerHandle(state_handle);

  hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle("Test_CC"), &cmd);
  jnt_vel_interface.registerHandle(vel_handle);

  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_vel_interface);
}

#endif