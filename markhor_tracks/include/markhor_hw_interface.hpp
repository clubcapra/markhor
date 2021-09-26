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

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[NUM_JOINTS];
  double pos[NUM_JOINTS];
  double vel[NUM_JOINTS];
  double eff[NUM_JOINTS];

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
};
#endif  // MARKHOR_HW_INTERFACE_H