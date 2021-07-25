#include <markhor_hw_interface_flipper.hpp>

MarkhorHWInterfaceFlipper::MarkhorHWInterfaceFlipper()
{
  joint_names_.push_back("flipper_fl_j");
  joint_names_.push_back("flipper_fr_j");
  joint_names_.push_back("flipper_rl_j");
  joint_names_.push_back("flipper_rr_j");

  // Status
  joint_position_.resize(NUM_JOINTS, 0.0);
  joint_velocity_.resize(NUM_JOINTS, 0.0);
  joint_effort_.resize(NUM_JOINTS, 0.0);

  // Command
  joint_position_command_.resize(NUM_JOINTS, 0.0);
  joint_velocity_command_.resize(NUM_JOINTS, 0.0);
  joint_effort_command_.resize(NUM_JOINTS, 0.0);

  setupRosControl();
  // setupCTREDrive();
}

void MarkhorHWInterfaceFlipper::setupRosControl()
{
  // connect and register the joint state and velocity interfaces
  for (unsigned int joint_id = 0; joint_id < NUM_JOINTS; ++joint_id)
  {
    hardware_interface::JointStateHandle state_handle(joint_names_[joint_id], &joint_position_[joint_id],
                                                      &joint_velocity_[joint_id], &joint_effort_[joint_id]);
    joint_state_interface_.registerHandle(state_handle);

    hardware_interface::JointHandle joint_handle_effort = hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_position_command_[joint_id]);
    position_joint_interface_.registerHandle(joint_handle_effort);
    // hardware_interface::JointHandle eff_handle(jnt_state_interface.getHandle(joint_names_[joint_id]),
    // &cmd[joint_id]); jnt_eff_interface.registerHandle(eff_handle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
}

void MarkhorHWInterfaceFlipper::setupCTREDrive()
{
  std::string interface = "can0";
  ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

  int drive_fl_id, drive_fr_id, drive_rl_id, drive_rr_id = 0;

  if (nh.getParam("/markhor/markhor_base_flipper/front_left", drive_fl_id) == true)
  {
    front_left_drive = std::make_unique<TalonSRX>(drive_fl_id);
  }
  if (nh.getParam("/markhor/markhor_base_flipper/front_right", drive_fr_id) == true)
  {
    front_right_drive = std::make_unique<TalonSRX>(drive_fr_id);
  }
  if (nh.getParam("/markhor/markhor_base_flipper/rear_left", drive_rl_id) == true)
  {
    rear_left_drive = std::make_unique<TalonSRX>(drive_rl_id);
  }
  if (nh.getParam("/markhor/markhor_base_flipper/rear_right", drive_rr_id) == true)
  {
    rear_right_drive = std::make_unique<TalonSRX>(drive_rr_id);
  }
}

void MarkhorHWInterfaceFlipper::write()
{
  ROS_INFO("HWI FLIPPER WRITE");
  ROS_INFO("position FL command : %f", joint_position_command_[0]);
  ROS_INFO("position FR command : %f", joint_position_command_[1]);
  ROS_INFO("position RL command : %f", joint_position_command_[2]);
  ROS_INFO("position RR command : %f", joint_position_command_[3]);

  // double diff_ang_speed_front_left = cmd[0];
  // double diff_ang_speed_rear_left = cmd[1];
  // double diff_ang_speed_front_right = cmd[2];
  // double diff_ang_speed_rear_right = cmd[3];

  // limitDifferentialSpeed(diff_ang_speed_front_left, diff_ang_speed_rear_left, diff_ang_speed_front_right,
  //                        diff_ang_speed_rear_right);

  // ctre::phoenix::unmanaged::FeedEnable(100);

  // // Set data
  // front_left_track_vel_msg.data = diff_ang_speed_front_left;
  // front_right_track_vel_msg.data = diff_ang_speed_front_right;
  // rear_left_track_vel_msg.data = diff_ang_speed_rear_left;
  // rear_right_track_vel_msg.data = diff_ang_speed_rear_right;

  // // Write to drive
  // front_left_drive->Set(ControlMode::PercentOutput, front_left_track_vel_msg.data);
  // front_right_drive->Set(ControlMode::PercentOutput, front_left_track_vel_msg.data);
  // rear_left_drive->Set(ControlMode::PercentOutput, front_left_track_vel_msg.data);
  // rear_right_drive->Set(ControlMode::PercentOutput, front_left_track_vel_msg.data);

  // // Publish data
  // front_left_track_vel_pub_.publish(front_left_track_vel_msg);
  // front_right_track_vel_pub_.publish(front_right_track_vel_msg);
  // rear_left_track_vel_pub_.publish(rear_left_track_vel_msg);
  // rear_right_track_vel_pub_.publish(rear_right_track_vel_msg);
}

void MarkhorHWInterfaceFlipper::read()
{
  // Read from the motor API, going to read from the TalonSRX objects
  ROS_INFO("HWI FLIPPER READs");
}

// ros::Time MarkhorHWInterfaceFlipper::get_time()
// {
//   prev_update_time = curr_update_time;
//   curr_update_time = ros::Time::now();
//   return curr_update_time;
// }

// ros::Duration MarkhorHWInterfaceFlipper::get_period()
// {
//   return curr_update_time - prev_update_time;
// }

// bool MarkhorHWInterfaceFlipper::start_callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
// {
//   running_ = true;
//   return true;
// }

// bool MarkhorHWInterfaceFlipper::stop_callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
// {
//   running_ = false;
//   return true;
// }

// void MarkhorHWInterfaceFlipper::limitDifferentialSpeed(double& diff_speed_front_left, double& diff_speed_rear_left,
//                                                 double& diff_speed_front_right, double& diff_speed_rear_right)
// {
//   // std::max can take a list to find the max value inside.
//   double speed = std::max({ std::abs(diff_speed_front_left), std::abs(diff_speed_rear_left),
//                             std::abs(diff_speed_front_right), std::abs(diff_speed_rear_right) });
//   if (speed > max_speed)
//   {
//     diff_speed_front_left *= max_speed / speed;
//     diff_speed_rear_left *= max_speed / speed;
//     diff_speed_front_right *= max_speed / speed;
//     diff_speed_rear_right *= max_speed / speed;
//   }
// }