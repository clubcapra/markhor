#include <markhor_hw_interface_flipper.hpp>
#include <string>
#include <unistd.h>

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

  nh.getParam("/markhor/markhor_base_flipper_node/config_folder_location", config_folder_str);
  nh.getParam("/markhor/markhor_base_flipper_node/config_file_1", config_file_1);
  nh.getParam("/markhor/markhor_base_flipper_node/config_file_2", config_file_2);
  nh.getParam("/markhor/markhor_base_flipper_node/config_file_3", config_file_3);
  uint count = 0;
  while (1)
  {
    if (count % 3 == 0)
    {
      file.open((config_folder_str + config_file_1).c_str(), std::fstream::out | std::fstream::trunc);
      if (!file)
      {
        ROS_INFO("error openning file %s", config_file_1.c_str());
      }
      file << "test : " << count++ << std::endl;
      file.rdbuf()->pubsync();
    }
    else if (count % 3 == 1)
    {
      file.open((config_folder_str + config_file_2).c_str(), std::fstream::out | std::fstream::trunc);
      if (!file)
      {
        ROS_INFO("error openning file %s", config_file_2.c_str());
      }
      file << "test2 : " << count++ << std::endl;
      file.rdbuf()->pubsync();
    }
    else
    {
      file.open((config_folder_str + config_file_3).c_str(), std::fstream::out | std::fstream::trunc);
      if (!file)
      {
        ROS_INFO("error openning file %s", config_file_3.c_str());
      }
      file << "test3 : " << count++ << std::endl;
      file.rdbuf()->pubsync();
    }

    file.close();
  }
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

  // if (nh.getParam("/markhor/markhor_base_flipper_node/front_left", drive_fl_id) == true)
  // {
  //   ROS_INFO("CREATE FRONT LEFT");
  //   front_left_drive = std::make_unique<TalonSRX>(drive_fl_id);
  //   front_left_drive->ConfigFactoryDefault();
  //   front_left_drive->GetSensorCollection().SetPulseWidthPosition(0, 30);
  // int error;
  // do
  // {
  //   error = front_left_drive->GetSensorCollection().SetPulseWidthPosition(0, 30); // TODO à changer
  //   ROS_INFO("SetPulseWidthPosition error code : %d", error);
  // } while (error == 0);
  //   front_left_drive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 50);
  //   front_left_drive->SetSensorPhase(true);
  //   front_left_drive->ConfigNominalOutputForward(0, kTimeoutMs);
  //   front_left_drive->ConfigNominalOutputReverse(0, kTimeoutMs);
  //   front_left_drive->ConfigPeakOutputForward(0.3, kTimeoutMs);
  //   front_left_drive->ConfigPeakOutputReverse(-0.3, kTimeoutMs);

  //   float kP, kI, kD = 0.0;
  //   nh.getParam("/markhor/markhor_base_flipper_node/kP", kP);
  //   nh.getParam("/markhor/markhor_base_flipper_node/kI", kI);
  //   nh.getParam("/markhor/markhor_base_flipper_node/kD", kD);

  //   front_left_drive->SelectProfileSlot(0, 0);
  //   front_left_drive->Config_kF(0, 0, 30);
  //   front_left_drive->Config_kP(0, kP, 30);
  //   front_left_drive->Config_kI(0, kI, 30);
  //   front_left_drive->Config_kD(0, kD, 30);
  // }
  if (nh.getParam("/markhor/markhor_base_flipper_node/front_right", drive_fr_id) == true)
  {
    ROS_INFO("CREATE FRONT RIGHT");
    // front_right_drive = std::make_unique<TalonSRX>(drive_fr_id);
    // front_right_drive->ConfigFactoryDefault();
    // front_right_drive->GetSensorCollection().SetPulseWidthPosition(0, 30);
    // int error;
    // do
    // {
    //   error = front_left_drive->GetSensorCollection().SetPulseWidthPosition(0, 30); // TODO à changer
    //   ROS_INFO("SetPulseWidthPosition error code : %d", error);
    // } while (error == 0);
    // front_right_drive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 50);
    // front_right_drive->SetSensorPhase(true);
    // front_right_drive->ConfigNominalOutputForward(0, kTimeoutMs);
    // front_right_drive->ConfigNominalOutputReverse(0, kTimeoutMs);
    // front_right_drive->ConfigPeakOutputForward(0.3, kTimeoutMs);
    // front_right_drive->ConfigPeakOutputReverse(-0.3, kTimeoutMs);

    // float kP, kI, kD = 0.0;
    // nh.getParam("/markhor/markhor_base_flipper_node/kP", kP);
    // nh.getParam("/markhor/markhor_base_flipper_node/kI", kI);
    // nh.getParam("/markhor/markhor_base_flipper_node/kD", kD);

    // front_right_drive->SelectProfileSlot(0, 0);
    // front_right_drive->Config_kF(0, 0, 30);
    // front_right_drive->Config_kP(0, kP, 30);
    // front_right_drive->Config_kI(0, kI, 30);
    // front_right_drive->Config_kD(0, kD, 30);
  }
  if (nh.getParam("/markhor/markhor_base_flipper_node/rear_left", drive_rl_id) == true)
  {
    ROS_INFO("CREATE REAR LEFT");
    rear_left_drive = std::make_unique<TalonSRX>(drive_rl_id);
    rear_left_drive->ConfigFactoryDefault();
    int absolutePosition = rear_left_drive->GetSensorCollection().GetPulseWidthPosition();
    int error;
    do
    {
      error = rear_left_drive->GetSensorCollection().SetPulseWidthPosition(-81925, 30);
      ROS_INFO("SetPulseWidthPosition error code : %d", error);
    } while (error != ErrorCode::OKAY);
    rear_left_drive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 50);
    rear_left_drive->SetSensorPhase(true);
    rear_left_drive->ConfigNominalOutputForward(0, kTimeoutMs);
    rear_left_drive->ConfigNominalOutputReverse(0, kTimeoutMs);
    rear_left_drive->ConfigPeakOutputForward(0.3, kTimeoutMs);
    rear_left_drive->ConfigPeakOutputReverse(-0.3, kTimeoutMs);

    float kP, kI, kD = 0.0;
    nh.getParam("/markhor/markhor_base_flipper_node/kP", kP);
    nh.getParam("/markhor/markhor_base_flipper_node/kI", kI);
    nh.getParam("/markhor/markhor_base_flipper_node/kD", kD);

    rear_left_drive->SelectProfileSlot(0, 0);
    rear_left_drive->Config_kF(0, 0, 30);
    rear_left_drive->Config_kP(0, kP, 30);
    rear_left_drive->Config_kI(0, kI, 30);
    rear_left_drive->Config_kD(0, kD, 30);

    nh.getParam("/markhor/markhor_base_flipper_node/rear_left_drive_upper_limit", rear_left_drive_upper_limit);
    nh.getParam("/markhor/markhor_base_flipper_node/rear_left_drive_lower_limit", rear_left_drive_lower_limit);
  }
  // if (nh.getParam("/markhor/markhor_base_flipper_node/rear_right", drive_rr_id) == true)
  // {
  //   rear_right_drive = std::make_unique<TalonSRX>(drive_rr_id);
  // }

  nh.getParam("/markhor/markhor_base_flipper_node/ratio", ratio);
}

void MarkhorHWInterfaceFlipper::write()
{
  // ROS_INFO("HWI FLIPPER WRITE");
  // ROS_INFO("position FL command : %f", joint_position_command_[0]);
  // ROS_INFO("position FR command : %f", joint_position_command_[1]);
  ROS_INFO("position RL command : %f", joint_position_command_[2]);
  ROS_INFO("RL lower limit : %f", rear_left_drive_lower_limit);
  ROS_INFO("RL upper limit : %f", rear_left_drive_upper_limit);
  // ROS_INFO("position RR command : %f", joint_position_command_[3]);

  ctre::phoenix::unmanaged::FeedEnable(100);

  // // Write to drive
  // front_left_drive->Set(ControlMode::Position, joint_position_command_[0]);
  // front_right_drive->Set(ControlMode::Position, joint_position_command_[1]);
  // printDriveInfo(front_right_drive);
  printDriveInfo(rear_left_drive);
  if (joint_position_command_[2] >= rear_left_drive_lower_limit &&
      joint_position_command_[2] < rear_left_drive_upper_limit)
  {
    rear_left_drive->Set(ControlMode::Position, joint_position_command_[2]);
  }
  saveDrivePosition(rear_left_drive);
  // front_right_drive->Set(ControlMode::Position, joint_position_command_[1]);
  // rear_left_drive->Set(ControlMode::Position, front_left_track_vel_msg.data);
  // rear_right_drive->Set(ControlMode::Position, front_left_track_vel_msg.data);
}

void MarkhorHWInterfaceFlipper::read()
{
  // Read from the motor API, going to read from the TalonSRX objects
  // ROS_INFO("HWI FLIPPER READs");
}

void MarkhorHWInterfaceFlipper::printDriveInfo(std::unique_ptr<TalonSRX>& drive)
{
  // ROS_INFO("GetPulseWidthPosition %d", drive->GetSensorCollection().GetPulseWidthPosition() & 0xFFF);
  ROS_INFO("GetPulseWidthPosition %d", drive->GetSensorCollection().GetPulseWidthPosition());
  ROS_INFO("GetClosedLoopError %d", drive->GetClosedLoopError(0) / ratio);
  ROS_INFO("GetClosedLoopTarget %f", drive->GetClosedLoopTarget(0));
}

void MarkhorHWInterfaceFlipper::saveDrivePosition(std::unique_ptr<TalonSRX>& drive)
{
}