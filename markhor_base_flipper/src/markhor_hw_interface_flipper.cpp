#include <markhor_hw_interface_flipper.hpp>

MarkhorHWInterfaceFlipper::MarkhorHWInterfaceFlipper()
{
  joint_names_.push_back("flipper_fl_j");
  joint_names_.push_back("flipper_fr_j");
  joint_names_.push_back("flipper_rl_j");
  joint_names_.push_back("flipper_rr_j");

  num_joints = joint_names_.size();

  // Status
  joint_position_.resize(num_joints, 0.0);
  joint_velocity_.resize(num_joints, 0.0);
  joint_effort_.resize(num_joints, 0.0);

  // Command
  joint_position_command_.resize(num_joints, 0.0);
  joint_velocity_command_.resize(num_joints, 0.0);
  joint_effort_command_.resize(num_joints, 0.0);

  setupRosControl();
  // setupCTREDrive();
}

void MarkhorHWInterfaceFlipper::setupRosControl()
{
  // connect and register the joint state and effort interfaces
  for (unsigned int joint_id = 0; joint_id < num_joints; ++joint_id)
  {
    hardware_interface::JointStateHandle state_handle(joint_names_[joint_id], &joint_position_[joint_id],
                                                      &joint_velocity_[joint_id], &joint_effort_[joint_id]);

    joint_state_interface_.registerHandle(state_handle);

    hardware_interface::JointHandle joint_handle_effort = hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_effort_command_[joint_id]);

    effort_joint_interface_.registerHandle(joint_handle_effort);

    registerInterface(&joint_state_interface_);
    registerInterface(&effort_joint_interface_);
  }
}

void MarkhorHWInterfaceFlipper::setupCTREDrive()
{
  std::string interface = "can0";
  ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

  int drive_fl_id, drive_fr_id, drive_rl_id, drive_rr_id = 0;

  if (nh.getParam("/markhor/markhor_base_node/front_left", drive_fl_id) == true)
  {
    front_left_drive = std::make_unique<TalonSRX>(drive_fl_id);
  }
  if (nh.getParam("/markhor/markhor_base_node/rear_left", drive_rl_id) == true)
  {
    rear_left_drive = std::make_unique<TalonSRX>(drive_rl_id);
  }
  if (nh.getParam("/markhor/markhor_base_node/front_right", drive_fr_id) == true)
  {
    front_right_drive = std::make_unique<TalonSRX>(drive_fr_id);
  }
  if (nh.getParam("/markhor/markhor_base_node/rear_right", drive_rr_id) == true)
  {
    rear_right_drive = std::make_unique<TalonSRX>(drive_rr_id);
  }
}

void MarkhorHWInterfaceFlipper::write()
{
  ROS_INFO("HWI FLIPPER WRITE");
  ROS_INFO("effort command for FL : %f", joint_effort_command_[0]);
  ROS_INFO("effort command for FR : %f", joint_effort_command_[1]);
  ROS_INFO("effort command for RL : %f", joint_effort_command_[2]);
  ROS_INFO("effort command for RR : %f", joint_effort_command_[3]);

  // Convert value of 0.0 .. -3.14 to 0 .. 100.

  ctre::phoenix::unmanaged::FeedEnable(100);

  // // Write to drive
  // front_left_drive->Set(ControlMode::PercentOutput, front_left_track_vel_msg.data);
  // front_right_drive->Set(ControlMode::PercentOutput, front_left_track_vel_msg.data);
  // rear_left_drive->Set(ControlMode::PercentOutput, front_left_track_vel_msg.data);
  // rear_right_drive->Set(ControlMode::PercentOutput, front_left_track_vel_msg.data);
}

void MarkhorHWInterfaceFlipper::read()
{
  // Read from the motor API, going to read from the TalonSRX objects
  ROS_INFO("HWI FLIPPER READs");
}