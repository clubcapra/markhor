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
#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"
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
#include <iostream>
#include <fstream>

constexpr unsigned int NUM_JOINTS = 4;

class MarkhorHWInterfaceFlipper : public hardware_interface::RobotHW
{
public:
  MarkhorHWInterfaceFlipper();
  void write();
  void read();

  ros::NodeHandle nh;
  int num_joints = 4;

private:
  void setupRosControl();
  void setupCTREDrive();
  void printDriveInfo(std::unique_ptr<TalonSRX>& drive);
  void saveDrivePosition(std::unique_ptr<TalonSRX>& drive);

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
  float rear_left_drive_upper_limit, rear_left_drive_lower_limit;
  std::unique_ptr<TalonSRX> rear_right_drive;
  int ratio = 1;

  std::fstream file;
  std::string config_file_1;
  std::string config_file_2;
  std::string config_file_3;
  std::string config_folder_str;
};
#endif  // MARKHOR_HW_INTERFACE_H