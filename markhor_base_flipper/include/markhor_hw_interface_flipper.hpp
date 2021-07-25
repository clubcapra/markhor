#ifndef MARKHOR_HW_INTERFACE_H
#define MARKHOR_HW_INTERFACE_H

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// ostringstream
#include <sstream>
#include <string>

// CTRE
#include "Platform-linux-socket-can.h"
#include "ctre/Phoenix.h"
#include "ctre/phoenix/MotorControl/CAN/TalonSRX.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

// STD
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

constexpr unsigned int NUM_JOINTS = 4;

class MarkhorHWInterfaceFlipper : public hardware_interface::RobotHW
{
public:
  MarkhorHWInterfaceFlipper();
  void write();
  void read();

  ros::NodeHandle nh;
  int num_joints = 0;

private:
  void setupRosControl();
  void setupCTREDrive();
  float normalizePosition();

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  std::vector<std::string> joint_names_;

  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;

  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;

  std::unique_ptr<TalonSRX> front_left_drive;
  std::unique_ptr<TalonSRX> front_right_drive;
  std::unique_ptr<TalonSRX> rear_left_drive;
  std::unique_ptr<TalonSRX> rear_right_drive;
};
#endif  // MARKHOR_HW_INTERFACE_H