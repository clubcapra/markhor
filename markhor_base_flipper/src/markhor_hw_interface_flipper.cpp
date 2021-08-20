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
  setupCTREDrive();
}

void MarkhorHWInterfaceFlipper::setupRosControl()
{
  // connect and register the joint state and effort interfaces
  for (unsigned int joint_id = 0; joint_id < num_joints; ++joint_id)
  {
    hardware_interface::JointStateHandle state_handle(joint_names_[joint_id], &joint_position_[joint_id],
                                                      &joint_velocity_[joint_id], &joint_effort_[joint_id]);

    joint_state_interface_.registerHandle(state_handle);

    hardware_interface::JointHandle joint_handle_position = hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_position_command_[joint_id]);

    position_joint_interface_.registerHandle(joint_handle_position);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
}

void MarkhorHWInterfaceFlipper::setupCTREDrive()
{
  std::string interface = "can0";
  ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

  const int kTimeoutMs = 30;

  int drive_fl_id, drive_fr_id, drive_rl_id, drive_rr_id = 0;

  if (nh.getParam("/markhor/markhor_base_flipper_node/front_left", drive_fl_id) == true)
  {
    ROS_INFO("CREATE FRONT LEFT");
    front_left_drive = std::make_unique<TalonSRX>(drive_fl_id);
    front_left_drive->ConfigFactoryDefault();
    front_left_drive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 50);
    front_left_drive->SetSensorPhase(false);
    front_left_drive->SelectProfileSlot(0, 0);
    front_left_drive->Config_kF(0, 0, 30);
    front_left_drive->Config_kP(0, 100.0, 30);
    front_left_drive->Config_kI(0, 0, 30);
    front_left_drive->Config_kD(0, 0, 30);
  }
  if (nh.getParam("/markhor/markhor_base_flipper_node/front_right", drive_fr_id) == true)
  {
    ROS_INFO("CREATE FRONT RIGHT");
    front_right_drive = std::make_unique<TalonSRX>(drive_fr_id);

    front_right_drive->ConfigFactoryDefault();

    if (front_right_drive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 50) != 0)
    {
      ROS_INFO("ConfigSelectedFeedbackSensor failed");
    }
    front_right_drive->SetSensorPhase(false);
    front_right_drive->SelectProfileSlot(0, 0);
    front_right_drive->Config_kF(0, 0, 30);
    front_right_drive->Config_kP(0, 1.0, 30);
    front_right_drive->Config_kI(0, 0, 30);
    front_right_drive->Config_kD(0, 0, 30);

    // ROS_INFO("Get front_right_drive PID config : %s", front_right_drive_pid.toString());
  }
  // if (nh.getParam("/markhor/markhor_base_flipper/rear_left", drive_rl_id) == true)
  // {
  //   rear_left_drive = std::make_unique<TalonSRX>(drive_rl_id);
  // }
  // if (nh.getParam("/markhor/markhor_base_flipper/rear_right", drive_rr_id) == true)
  // {
  //   rear_right_drive = std::make_unique<TalonSRX>(drive_rr_id);
  // }
}

void MarkhorHWInterfaceFlipper::write()
{
  // ROS_INFO("HWI FLIPPER WRITE");
  // ROS_INFO("position FL command : %f", joint_position_command_[0]);
  // ROS_INFO("position FR command : %f", joint_position_command_[1]);
  // ROS_INFO("position RL command : %f", joint_position_command_[2]);
  // ROS_INFO("position RR command : %f", joint_position_command_[3]);

  // double diff_ang_speed_front_left = cmd[0];
  // double diff_ang_speed_rear_left = cmd[1];
  // double diff_ang_speed_front_right = cmd[2];
  // double diff_ang_speed_rear_right = cmd[3];

  // limitDifferentialSpeed(diff_ang_speed_front_left, diff_ang_speed_rear_left, diff_ang_speed_front_right,
  //                        diff_ang_speed_rear_right);

  ctre::phoenix::unmanaged::FeedEnable(100);

  // // Write to drive
  // front_left_drive->Set(ControlMode::Position, joint_position_command_[0]);
  ROS_INFO("GetPulseWidthPosition %d", front_left_drive->GetSensorCollection().GetPulseWidthPosition() & 0xFFF);
  ROS_INFO("GetClosedLoopError %lf", front_left_drive->GetClosedLoopError(0));
  ROS_INFO("GetClosedLoopTarget %lf", front_left_drive->GetClosedLoopTarget(0));
  if (joint_position_command_[0] != 0.0)
  {
    front_left_drive->Set(ControlMode::Position, joint_position_command_[0]);
  }
  ROS_INFO("GetPulseWidthPosition %d", front_left_drive->GetSensorCollection().GetPulseWidthPosition() & 0xFFF);
  ROS_INFO("GetClosedLoopError %lf", front_left_drive->GetClosedLoopError(0));
  ROS_INFO("GetClosedLoopTarget %lf", front_left_drive->GetClosedLoopTarget(0));
  // front_right_drive->Set(ControlMode::Position, joint_position_command_[1]);
  // rear_left_drive->Set(ControlMode::PercentOutput, front_left_track_vel_msg.data);
  // rear_right_drive->Set(ControlMode::PercentOutput, front_left_track_vel_msg.data);
}

void MarkhorHWInterfaceFlipper::read()
{
  // Read from the motor API, going to read from the TalonSRX objects
  // ROS_INFO("HWI FLIPPER READs");
}