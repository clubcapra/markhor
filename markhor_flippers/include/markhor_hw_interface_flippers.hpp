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

class MarkhorHWInterfaceFlippers : public hardware_interface::RobotHW
{
public:
  MarkhorHWInterfaceFlippers();
  void write();
  void read();

  ros::NodeHandle nh_;
  int num_joints_ = 0;

  int current_limit_ = 30;
  
private:
  void setupRosControl();
  void setupCtreDrive();
  bool hasResetOccurred();
  void printDriveInfo(std::unique_ptr<TalonSRX>& drive);
  
  void saveDrivePosition();
  void writeDrivePositionToFile(std::string config_file_name);

  void loadDrivePosition();
  void readDrivePositionFromFile(std::string config_file_name, std::fstream& config_file);
  void parseDrivePosition(std::string line);
  void applyDrivePosition(std::unique_ptr<TalonSRX>& drive, float drive_position);

  int getEncoderPosition(std::unique_ptr<TalonSRX>& drive);

  const int timeout_ms_ = 30;
  
  int drive_fl_id_, drive_fr_id_, drive_rl_id_, drive_rr_id_ = 0;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  std::vector<std::string> joint_names_;

  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;

  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;

  std::unique_ptr<TalonSRX> front_left_drive_;
  float front_left_drive_upper_limit_;
  float front_left_drive_lower_limit_;
  std::unique_ptr<TalonSRX> front_right_drive_;
  float front_right_drive_upper_limit_;
  float front_right_drive_lower_limit_;
  std::unique_ptr<TalonSRX> rear_left_drive_;
  float rear_left_drive_upper_limit_;
  float rear_left_drive_lower_limit_;
  std::unique_ptr<TalonSRX> rear_right_drive_;
  float rear_right_drive_upper_limit_;
  float rear_right_drive_lower_limit_;

  std::fstream drive_config_file_;
  std::string config_file_1_;
  std::string config_file_2_;
  std::string config_folder_str_;

  bool alternating_value_ = true;

  float front_left_drive_base_position_;
  float front_right_drive_base_position_;
  float rear_left_drive_base_position_;
  float rear_right_drive_base_position_;
  bool has_reset_event_occured_ = false;
  float accumulator_fr_ = 0;
  float accumulator_fl_ = 0;
  float accumulator_rl_ = 0;
  float accumulator_rr_ = 0;
};
#endif  // MARKHOR_HW_INTERFACE_H