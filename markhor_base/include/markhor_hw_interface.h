#ifndef CAPRA_CONTROL_H
#define CAPRA_CONTROL_H
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

#include "ctre/Phoenix.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "Platform-linux-socket-can.h"
#include "ctre/phoenix/MotorControl/CAN/TalonSRX.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <memory>
#include <map>

const unsigned int NUM_JOINTS = 4;

class MarkhorHWInterface : public hardware_interface::RobotHW
{
public:
  MarkhorHWInterface();

  void write()
  {
    double diff_ang_speed_front_left = cmd[0];
    double diff_ang_speed_rear_left = cmd[1];
    double diff_ang_speed_front_right = cmd[2];
    double diff_ang_speed_rear_right = cmd[3];
    limitDifferentialSpeed(diff_ang_speed_front_left, diff_ang_speed_rear_left, diff_ang_speed_front_right,
                           diff_ang_speed_rear_right);
    // Set data
    front_left_track_vel_msg.data = diff_ang_speed_front_left;
    rear_left_track_vel_msg.data = diff_ang_speed_rear_left;
    front_right_track_vel_msg.data = diff_ang_speed_front_right;
    rear_right_track_vel_msg.data = diff_ang_speed_rear_right;

    // TODO
    // write to the motor API

    // Publish data
    front_left_track_vel_pub_.publish(front_left_track_vel_msg);
    rear_left_track_vel_pub_.publish(rear_left_track_vel_msg);
    front_right_track_vel_pub_.publish(front_right_track_vel_msg);
    rear_right_track_vel_pub_.publish(rear_right_track_vel_msg);
  }

  void read(const ros::Duration& period)
  {
  }

  ros::Time get_time()
  {
    prev_update_time = curr_update_time;
    curr_update_time = ros::Time::now();
    return curr_update_time;
  }

  ros::Duration get_period()
  {
    return curr_update_time - prev_update_time;
  }

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

private:
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

  bool start_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = false;
    return true;
  }

  void limitDifferentialSpeed(double& diff_speed_front_left, double& diff_speed_rear_left,
                              double& diff_speed_front_right, double& diff_speed_rear_right)
  {
    // std::max can take a list to find the max value inside.
    double speed = std::max({ std::abs(diff_speed_front_left), std::abs(diff_speed_rear_left),
                              std::abs(diff_speed_front_right), std::abs(diff_speed_rear_right) });
    if (speed > max_speed)
    {
      diff_speed_front_left *= max_speed / speed;
      diff_speed_rear_left *= max_speed / speed;
      diff_speed_front_right *= max_speed / speed;
      diff_speed_rear_right *= max_speed / speed;
    }
  }

};  // class

MarkhorHWInterface::MarkhorHWInterface()
  : running_(true)
  , private_nh("~")
  , start_srv_(nh.advertiseService("start", &MarkhorHWInterface::start_callback, this))
  , stop_srv_(nh.advertiseService("stop", &MarkhorHWInterface::start_callback, this))
{
  private_nh.param<double>("max_speed", max_speed, 1.0);

  std::fill_n(this->pos, NUM_JOINTS, 0.0);
  std::fill_n(this->vel, NUM_JOINTS, 0.0);
  std::fill_n(this->eff, NUM_JOINTS, 0.0);
  std::fill_n(this->cmd, NUM_JOINTS, 0.0);

  std::string tracks[NUM_JOINTS] = { "flipper_fr_motor_j", "flipper_rr_motor_j", "flipper_fl_motor_j",
                                     "flipper_rl_motor_j" };
  // connect and register the joint state and velocity interfaces
  for (unsigned int i = 0; i < NUM_JOINTS; ++i)
  {
    hardware_interface::JointStateHandle state_handle(tracks[i], &pos[i], &vel[i], &eff[i]);
    jnt_state_interface.registerHandle(state_handle);

    hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(tracks[i]), &cmd[i]);
    jnt_vel_interface.registerHandle(vel_handle);
  }
  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_vel_interface);

  // Initialize publishers and subscribers
  front_left_track_vel_pub_ = nh.advertise<std_msgs::Float32>("front_left_track_vel", 1);
  rear_left_track_vel_pub_ = nh.advertise<std_msgs::Float32>("rear_right_track_vel", 1);
  front_right_track_vel_pub_ = nh.advertise<std_msgs::Float32>("front_right_track_vel", 1);
  rear_right_track_vel_pub_ = nh.advertise<std_msgs::Float32>("rear_track_vel", 1);
}
#endif