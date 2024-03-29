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

constexpr unsigned int NUM_JOINTS = 4;

class MarkhorHWInterface : public hardware_interface::RobotHW
{
public:
  MarkhorHWInterface();
  void write();
  void read(const ros::Duration& period);
  ros::Time get_time();
  ros::Duration get_period();

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

private:
  bool start_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/);
  bool stop_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/);
  void limitDifferentialSpeed(double& diff_speed_front_left, double& diff_speed_rear_left,
                              double& diff_speed_front_right, double& diff_speed_rear_right);

  void setupPublisher();
  void setupRosControl();
  void setupCTREDrive();
  void setupParam();
  
  const int timeout_ms_ = 30;

  int tracks_i_max = 0;
  int tracks_i_zone = 0;

  double tracks_kp = 0;
  double tracks_ki = 0;
  double tracks_kd = 0;

  double tracks_fb_coeff = 1;

  double track_encoder_reduction_coeff = 1.53846153846153;

  const int step_per_turn = 4096;

  int allowable_closedloop_error = 0;

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[NUM_JOINTS];
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;

  bool running_;
  double max_speed;
  ros::Time curr_update_time, prev_update_time;

  ros::Publisher front_left_track_vel_pub_;
  ros::Publisher front_right_track_vel_pub_;
  ros::Publisher rear_left_track_vel_pub_;
  ros::Publisher rear_right_track_vel_pub_;

  std_msgs::Float32 front_left_track_vel_msg;
  std_msgs::Float32 rear_left_track_vel_msg;
  std_msgs::Float32 front_right_track_vel_msg;
  std_msgs::Float32 rear_right_track_vel_msg;

  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;

  std::unique_ptr<TalonSRX> front_left_drive;
  std::unique_ptr<TalonSRX> front_right_drive;
  std::unique_ptr<TalonSRX> rear_left_drive;
  std::unique_ptr<TalonSRX> rear_right_drive;

  int log_throttle_speed = 1;
};
#endif  // MARKHOR_HW_INTERFACE_H
