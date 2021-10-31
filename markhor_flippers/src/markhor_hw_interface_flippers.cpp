#include <ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h>
#include <unistd.h>
#include <string>

#include "markhor_hw_interface_flippers.hpp"

MarkhorHWInterfaceFlippers::MarkhorHWInterfaceFlippers()
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
  setupCtreDrive();

  nh.getParam("/markhor/markhor_flippers_node/config_folder_location", config_folder_str);
  nh.getParam("/markhor/markhor_flippers_node/config_file_1", config_file_1);
  nh.getParam("/markhor/markhor_flippers_node/config_file_2", config_file_2);

  loadDrivePosition();
}

void MarkhorHWInterfaceFlippers::setupRosControl()
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

void MarkhorHWInterfaceFlippers::setupCtreDrive()
{
  std::string interface = "can0";
  ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

  SupplyCurrentLimitConfiguration current_limit_config;
  current_limit_config.enable = true;
  current_limit_config.currentLimit = this->currentLimit;

  float kP, kI, kD = 0.0;
  nh.getParam("/markhor/markhor_flippers_node/kP", kP);
  nh.getParam("/markhor/markhor_flippers_node/kI", kI);
  nh.getParam("/markhor/markhor_flippers_node/kD", kD);

  if (nh.getParam("/markhor/markhor_flippers_node/front_left", drive_fl_id) == true)
  {
    front_left_drive = std::make_unique<TalonSRX>(drive_fl_id);
    front_left_drive->ConfigFactoryDefault();
    front_left_drive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, this->kTimeoutMs);
    front_left_drive->SetSensorPhase(true);
    front_left_drive->ConfigSupplyCurrentLimit(current_limit_config);
    front_left_drive->ConfigNominalOutputForward(0, this->kTimeoutMs);
    front_left_drive->ConfigNominalOutputReverse(0, this->kTimeoutMs);
    front_left_drive->ConfigAllowableClosedloopError(0, 100, this->kTimeoutMs);

    double front_left_peak_output_forward, front_left_peak_output_reverse = 0;
    nh.getParam("/markhor/markhor_flippers_node/front_left_drive_peak_output_forward", front_left_peak_output_forward);
    nh.getParam("/markhor/markhor_flippers_node/front_left_drive_peak_output_reverse", front_left_peak_output_reverse);

    front_left_drive->ConfigPeakOutputForward(front_left_peak_output_forward, this->kTimeoutMs);
    front_left_drive->ConfigPeakOutputReverse(front_left_peak_output_reverse, this->kTimeoutMs);

    front_left_drive->SelectProfileSlot(0, 0);
    front_left_drive->Config_kF(0, 0, this->kTimeoutMs);
    front_left_drive->Config_kP(0, kP, this->kTimeoutMs);
    front_left_drive->Config_kI(0, kI, this->kTimeoutMs);
    front_left_drive->Config_kD(0, kD, this->kTimeoutMs);

    nh.getParam("/markhor/markhor_flippers_node/front_left_drive_upper_limit", front_left_drive_upper_limit);
    nh.getParam("/markhor/markhor_flippers_node/front_left_drive_lower_limit", front_left_drive_lower_limit);
  }
  if (nh.getParam("/markhor/markhor_flippers_node/front_right", drive_fr_id) == true)
  {
    front_right_drive = std::make_unique<TalonSRX>(drive_fr_id);
    front_right_drive->ConfigFactoryDefault();
    front_right_drive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 50);
    front_right_drive->SetSensorPhase(true);
    front_right_drive->ConfigSupplyCurrentLimit(current_limit_config);
    front_right_drive->ConfigNominalOutputForward(0, this->kTimeoutMs);
    front_right_drive->ConfigNominalOutputReverse(0, this->kTimeoutMs);
    front_right_drive->ConfigAllowableClosedloopError(0, 100, this->kTimeoutMs);

    double front_right_peak_output_forward, front_right_peak_output_reverse = 0;
    nh.getParam("/markhor/markhor_flippers_node/front_right_drive_peak_output_forward",
                front_right_peak_output_forward);
    nh.getParam("/markhor/markhor_flippers_node/front_right_drive_peak_output_reverse",
                front_right_peak_output_reverse);

    front_right_drive->ConfigPeakOutputForward(front_right_peak_output_forward, this->kTimeoutMs);
    front_right_drive->ConfigPeakOutputReverse(front_right_peak_output_reverse, this->kTimeoutMs);

    front_right_drive->SelectProfileSlot(0, 0);
    front_right_drive->Config_kF(0, 0, this->kTimeoutMs);
    front_right_drive->Config_kP(0, kP, this->kTimeoutMs);
    front_right_drive->Config_kI(0, kI, this->kTimeoutMs);
    front_right_drive->Config_kD(0, kD, this->kTimeoutMs);

    nh.getParam("/markhor/markhor_flippers_node/front_right_drive_upper_limit", front_right_drive_upper_limit);
    nh.getParam("/markhor/markhor_flippers_node/front_right_drive_lower_limit", front_right_drive_lower_limit);
  }
  if (nh.getParam("/markhor/markhor_flippers_node/rear_left", drive_rl_id) == true)
  {
    rear_left_drive = std::make_unique<TalonSRX>(drive_rl_id);
    rear_left_drive->ConfigFactoryDefault();
    rear_left_drive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 50);
    rear_left_drive->SetSensorPhase(true);

    double rear_left_peak_output_forward, rear_left_peak_output_reverse = 0;
    nh.getParam("/markhor/markhor_flippers_node/rear_left_drive_peak_output_forward", rear_left_peak_output_forward);
    nh.getParam("/markhor/markhor_flippers_node/rear_left_drive_peak_output_reverse", rear_left_peak_output_reverse);

    rear_left_drive->ConfigSupplyCurrentLimit(current_limit_config);
    rear_left_drive->ConfigNominalOutputForward(0, this->kTimeoutMs);
    rear_left_drive->ConfigNominalOutputReverse(0, this->kTimeoutMs);
    rear_left_drive->ConfigPeakOutputForward(rear_left_peak_output_forward, this->kTimeoutMs);
    rear_left_drive->ConfigPeakOutputReverse(rear_left_peak_output_reverse, this->kTimeoutMs);
    rear_left_drive->ConfigAllowableClosedloopError(0, 100, this->kTimeoutMs);

    rear_left_drive->SelectProfileSlot(0, 0);
    rear_left_drive->Config_kF(0, 0, this->kTimeoutMs);
    rear_left_drive->Config_kP(0, kP, this->kTimeoutMs);
    rear_left_drive->Config_kI(0, kI, this->kTimeoutMs);
    rear_left_drive->Config_kD(0, kD, this->kTimeoutMs);

    nh.getParam("/markhor/markhor_flippers_node/rear_left_drive_upper_limit", rear_left_drive_upper_limit);
    nh.getParam("/markhor/markhor_flippers_node/rear_left_drive_lower_limit", rear_left_drive_lower_limit);
  }
  if (nh.getParam("/markhor/markhor_flippers_node/rear_right", drive_rr_id) == true)
  {
    rear_right_drive = std::make_unique<TalonSRX>(drive_rr_id);
    rear_right_drive->ConfigFactoryDefault();
    rear_right_drive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 50);
    rear_right_drive->SetSensorPhase(true);
    rear_right_drive->ConfigSupplyCurrentLimit(current_limit_config);
    rear_right_drive->ConfigNominalOutputForward(0, this->kTimeoutMs);
    rear_right_drive->ConfigNominalOutputReverse(0, this->kTimeoutMs);

    double rear_right_peak_output_forward, rear_right_peak_output_reverse = 0;
    nh.getParam("/markhor/markhor_flippers_node/rear_right_drive_peak_output_forward", rear_right_peak_output_forward);
    nh.getParam("/markhor/markhor_flippers_node/rear_right_drive_peak_output_reverse", rear_right_peak_output_reverse);

    rear_right_drive->ConfigPeakOutputForward(rear_right_peak_output_forward, this->kTimeoutMs);
    rear_right_drive->ConfigPeakOutputReverse(rear_right_peak_output_reverse, this->kTimeoutMs);
    rear_right_drive->ConfigAllowableClosedloopError(0, 100, this->kTimeoutMs);

    rear_right_drive->SelectProfileSlot(0, 0);
    rear_right_drive->Config_kF(0, 0, this->kTimeoutMs);
    rear_right_drive->Config_kP(0, kP, this->kTimeoutMs);
    rear_right_drive->Config_kI(0, kI, this->kTimeoutMs);
    rear_right_drive->Config_kD(0, kD, this->kTimeoutMs);

    nh.getParam("/markhor/markhor_flippers_node/rear_right_drive_upper_limit", rear_right_drive_upper_limit);
    nh.getParam("/markhor/markhor_flippers_node/rear_right_drive_lower_limit", rear_right_drive_lower_limit);
  }
}

void MarkhorHWInterfaceFlippers::write()
{
  // Write to drive

  if (hasResetOccurred() == true)
  {
    loadDrivePosition();
    accumulator_fl = 0;
    accumulator_fr = 0;
    accumulator_rl = 0;
    accumulator_rr = 0;
  }

  ctre::phoenix::unmanaged::FeedEnable(100);

  printDriveInfo(front_left_drive);

  /*
  The lines :
    Lower limit :
    front_left_drive_lower_limit <= front_left_drive_base_position + accumulator_fl + joint_position_command_[0]

    And

    Upper limit :
    front_left_drive_base_position + accumulator_fl + joint_position_command_[0] < front_left_drive_upper_limit

  in each if condition for each drive, represent if the next position value we want to apply to the flipper is going to
  be above or under the limit of the flipper.
 */

  if (front_left_drive_lower_limit <= front_left_drive_base_position + accumulator_fl + joint_position_command_[0] &&
      front_left_drive_base_position + accumulator_fl + joint_position_command_[0] < front_left_drive_upper_limit)
  {
    accumulator_fl += joint_position_command_[0];
    float target = front_left_drive_base_position + accumulator_fl;
    ROS_INFO("target = [%f]", target);
    front_left_drive->Set(ControlMode::Position, target);
  }

  printDriveInfo(front_right_drive);
  if (front_right_drive_lower_limit <= front_right_drive_base_position + accumulator_fr + joint_position_command_[1] &&
      front_right_drive_base_position + accumulator_fr + joint_position_command_[1] < front_right_drive_upper_limit)
  {
    accumulator_fr += joint_position_command_[1];
    float target = front_right_drive_base_position + accumulator_fr;
    ROS_INFO("target = [%f]", target);
    front_right_drive->Set(ControlMode::Position, target);
  }

  printDriveInfo(rear_left_drive);
  if (rear_left_drive_lower_limit <= rear_left_drive_base_position + accumulator_rl + joint_position_command_[2] &&
      rear_left_drive_base_position + accumulator_rl + joint_position_command_[2] < rear_left_drive_upper_limit)
  {
    accumulator_rl += joint_position_command_[2];
    float target = rear_left_drive_base_position + accumulator_rl;
    ROS_INFO("target = [%f]", target);
    rear_left_drive->Set(ControlMode::Position, target);
  }

  printDriveInfo(rear_right_drive);
  if (rear_right_drive_lower_limit <= rear_right_drive_base_position + accumulator_rr + joint_position_command_[3] &&
      rear_right_drive_base_position + accumulator_rr + joint_position_command_[3] < rear_right_drive_upper_limit)
  {
    accumulator_rr += joint_position_command_[3];
    float target = rear_right_drive_base_position + accumulator_rr;
    ROS_INFO("target = [%f]", target);
    rear_right_drive->Set(ControlMode::Position, target);
  }
  saveDrivePosition();
}

void MarkhorHWInterfaceFlippers::read()
{
  // Read from the motor API, going to read from the TalonSRX objects
}

void MarkhorHWInterfaceFlippers::printDriveInfo(std::unique_ptr<TalonSRX>& drive)
{
  ROS_INFO("-------------");
  if (drive->GetDeviceID() == drive_fl_id)
  {
    ROS_INFO("position FL command : %f", joint_position_command_[0]);
    ROS_INFO("FL lower limit : %f", front_left_drive_lower_limit);
    ROS_INFO("FL upper limit : %f", front_left_drive_upper_limit);
    ROS_INFO("front_left_drive_base_position : %f", front_left_drive_base_position);
    ROS_INFO("accumulator_fl : %f", accumulator_fl);
    ROS_INFO("Target : %f", front_left_drive_base_position + accumulator_fl + joint_position_command_[0]);
    ROS_INFO("Drive %d : output voltage : %f", front_left_drive->GetDeviceID(),
             front_left_drive->GetMotorOutputVoltage());
  }
  else if (drive->GetDeviceID() == drive_fr_id)
  {
    ROS_INFO("position FR command : %f", joint_position_command_[1]);
    ROS_INFO("FR lower limit : %f", front_right_drive_lower_limit);
    ROS_INFO("FR upper limit : %f", front_right_drive_upper_limit);
    ROS_INFO("front_right_drive_base_position : %f", front_right_drive_base_position);
    ROS_INFO("accumulator_fr : %f", accumulator_fr);
    ROS_INFO("Target : %f", front_right_drive_base_position + accumulator_fr + joint_position_command_[1]);
    ROS_INFO("Drive %d : output voltage : %f", front_right_drive->GetDeviceID(),
             front_right_drive->GetMotorOutputVoltage());
  }
  else if (drive->GetDeviceID() == drive_rl_id)
  {
    ROS_INFO("position RL command : %f", joint_position_command_[2]);
    ROS_INFO("RL lower limit : %f", rear_left_drive_lower_limit);
    ROS_INFO("RL upper limit : %f", rear_left_drive_upper_limit);
    ROS_INFO("rear_left_drive_base_position : %f", rear_left_drive_base_position);
    ROS_INFO("accumulator_rl : %f", accumulator_rl);
    ROS_INFO("Target : %f", rear_left_drive_base_position + accumulator_rl + joint_position_command_[2]);
    ROS_INFO("Drive %d : output voltage : %f", rear_left_drive->GetDeviceID(),
             rear_left_drive->GetMotorOutputVoltage());
  }
  else if (drive->GetDeviceID() == drive_rr_id)
  {
    ROS_INFO("position RR command : %f", joint_position_command_[3]);
    ROS_INFO("RR lower limit : %f", rear_right_drive_lower_limit);
    ROS_INFO("RR upper limit : %f", rear_right_drive_upper_limit);
    ROS_INFO("rear_right_drive_base_position : %f", rear_right_drive_base_position);
    ROS_INFO("accumulator_rr : %f", accumulator_rl);
    ROS_INFO("Target : %f", rear_right_drive_base_position + accumulator_rr + joint_position_command_[3]);
    ROS_INFO("Drive %d : output voltage : %f", rear_right_drive->GetDeviceID(),
             rear_right_drive->GetMotorOutputVoltage());
  }
  else
  {
    ROS_INFO("Device ID is not found");
    return;
  }

  ROS_INFO("GetStatorCurrent %f", drive->GetStatorCurrent());
  ROS_INFO("GetPulseWidthPosition %d", drive->GetSensorCollection().GetPulseWidthPosition());
  ROS_INFO("GetClosedLoopError %d", drive->GetClosedLoopError(0));
  ROS_INFO("GetClosedLoopTarget %f", drive->GetClosedLoopTarget(0));
}

void MarkhorHWInterfaceFlippers::saveDrivePosition()
{
  // TODO : Only write if the value is different

  if (alternating_value == true)
  {
    alternating_value = false;
    writeDrivePositionToFile(config_file_1);
  }
  else
  {
    alternating_value = true;
    writeDrivePositionToFile(config_file_2);
  }
}

void MarkhorHWInterfaceFlippers::writeDrivePositionToFile(std::string config_file_name)
{
  drive_config_file.open((config_folder_str + config_file_name).c_str(), std::fstream::out | std::fstream::trunc);
  if (!drive_config_file)
  {
    ROS_ERROR("%s - Can't open file %s", __FUNCTION__, config_file_name.c_str());
    ros::shutdown();
    return;
  }

  if (front_left_drive)
  {
    drive_config_file << front_left_drive->GetDeviceID() << ":" << getEncoderPosition(front_left_drive) << std::endl;
  }
  if (front_right_drive)
  {
    drive_config_file << front_right_drive->GetDeviceID() << ":" << getEncoderPosition(front_right_drive) << std::endl;
  }
  if (rear_left_drive)
  {
    drive_config_file << rear_left_drive->GetDeviceID() << ":" << getEncoderPosition(rear_left_drive) << std::endl;
  }
  if (rear_right_drive)
  {
    drive_config_file << rear_right_drive->GetDeviceID() << ":" << getEncoderPosition(rear_right_drive) << std::flush;
  }
  drive_config_file.rdbuf()->pubsync();
  drive_config_file.close();
}

int MarkhorHWInterfaceFlippers::getEncoderPosition(std::unique_ptr<TalonSRX>& drive)
{
  int pulse_width_position = drive->GetSensorCollection().GetPulseWidthPosition();
  if (drive->GetDeviceID() == drive_fl_id)
  {
    if (pulse_width_position < -1 * front_left_drive_upper_limit)
    {
      return -1 * front_left_drive_upper_limit;
    }
    else if (pulse_width_position > -1 * front_left_drive_lower_limit)
    {
      return -1 * front_left_drive_lower_limit;
    }
    else
    {
      return pulse_width_position;
    }
  }
  else if (drive->GetDeviceID() == drive_fr_id)
  {
    if (pulse_width_position < -1 * front_right_drive_upper_limit)
    {
      return -1 * front_right_drive_upper_limit;
    }
    else if (pulse_width_position > -1 * front_right_drive_lower_limit)
    {
      return -1 * front_right_drive_lower_limit;
    }
    else
    {
      return pulse_width_position;
    }
  }
  else if (drive->GetDeviceID() == drive_rl_id)
  {
    if (pulse_width_position < -1 * rear_left_drive_upper_limit)
    {
      return -1 * rear_left_drive_upper_limit;
    }
    else if (pulse_width_position > -1 * rear_left_drive_lower_limit)
    {
      return -1 * rear_left_drive_lower_limit;
    }
    else
    {
      return pulse_width_position;
    }
  }
  else if (drive->GetDeviceID() == drive_rr_id)
  {
    if (pulse_width_position < -1 * rear_right_drive_upper_limit)
    {
      return -1 * rear_right_drive_upper_limit;
    }
    else if (pulse_width_position > -1 * rear_right_drive_lower_limit)
    {
      return -1 * rear_right_drive_lower_limit;
    }
    else
    {
      return pulse_width_position;
    }
  }
  else
  {
    ROS_FATAL("Could not find drive %d in drives list", drive->GetDeviceID());
    ros::shutdown();
    return 0;
  }
}

void MarkhorHWInterfaceFlippers::loadDrivePosition()
{
  bool is_drives_config_file_1_empty, is_drives_config_file_2_empty;

  // TODO : there's doesn't need to have two fstream object, but right now it doesn't work with only one
  std::fstream drive_config_file_1, drive_config_file_2;

  // Check if the files exists
  drive_config_file_1.open((config_folder_str + config_file_1).c_str(), std::fstream::in);
  drive_config_file_2.open((config_folder_str + config_file_2).c_str(), std::fstream::in);
  if (drive_config_file_1.good() == false || drive_config_file_2.good() == false)
  {
    ROS_FATAL("Missing drives calibration file(s). You need to either create the files or calibrate it before using "
              "markhor_flippers node");
    ros::shutdown();
    return;
  }

  // Check if the files are empty
  if (drive_config_file_1.peek() != std::fstream::traits_type::eof())
  {
    is_drives_config_file_1_empty = false;
  }
  else
  {
    is_drives_config_file_1_empty = true;
    ROS_WARN("drives config file 1 is empty");
  }

  if (is_drives_config_file_1_empty == true && drive_config_file_2.peek() != std::fstream::traits_type::eof())
  {
    is_drives_config_file_2_empty = false;
  }
  else
  {
    is_drives_config_file_2_empty = true;
    if (drive_config_file_2.peek() == std::fstream::traits_type::eof())
    {
      ROS_WARN("drives config file 2 is empty");
    }
  }

  if (is_drives_config_file_1_empty == true && is_drives_config_file_2_empty == true)
  {
    // TODO : Add link to documentation on how to calibrate the flipper and create the files
    ROS_FATAL("Drives configuration files are empty. Please calibrate before using markhor_flippers node");
    ros::shutdown();
    return;
  }

  // Read the files
  if (is_drives_config_file_1_empty == false)
  {
    readDrivePositionFromFile(config_file_1, drive_config_file_1);
  }
  if (is_drives_config_file_2_empty == false)
  {
    readDrivePositionFromFile(config_file_2, drive_config_file_2);
  }
  drive_config_file_1.close();
  drive_config_file_2.close();
}

void MarkhorHWInterfaceFlippers::readDrivePositionFromFile(std::string config_file_name, std::fstream& config_file)
{
  std::string line;
  while (std::getline(config_file, line))
  {
    parseDrivePosition(line);
  }
}

void MarkhorHWInterfaceFlippers::parseDrivePosition(std::string line)
{
  int drive_id;
  int drive_position;
  std::string::size_type pos = line.find(':');
  if (pos == std::string::npos)
  {
    ROS_ERROR("Could not find \':\' character in calibration line : %s", line.c_str());
    return;
  }
  drive_id = std::stoi(line.substr(0, pos));
  drive_position = std::stoi(line.substr(pos + 1, line.length()));

  if (drive_id == drive_fl_id)
  {
    applyDrivePosition(front_left_drive, drive_position);
  }
  if (drive_id == drive_fr_id)
  {
    applyDrivePosition(front_right_drive, drive_position);
  }
  if (drive_id == drive_rl_id)
  {
    applyDrivePosition(rear_left_drive, drive_position);
  }
  if (drive_id == drive_rr_id)
  {
    applyDrivePosition(rear_right_drive, drive_position);
  }

  HasResetEventOccurred = false;
}

void MarkhorHWInterfaceFlippers::applyDrivePosition(std::unique_ptr<TalonSRX>& drive, float drive_position)
{
  int error;
  if (HasResetEventOccurred == false)
  {
    if (drive->GetDeviceID() == drive_fl_id)
    {
      front_left_drive_base_position = -1 * drive_position;
    }
    else if (drive->GetDeviceID() == drive_fr_id)
    {
      front_right_drive_base_position = -1 * drive_position;
    }
    else if (drive->GetDeviceID() == drive_rl_id)
    {
      rear_left_drive_base_position = -1 * drive_position;
    }
    else if (drive->GetDeviceID() == drive_rr_id)
    {
      rear_right_drive_base_position = -1 * drive_position;
    }
  }
  do
  {
    error = drive->GetSensorCollection().SetPulseWidthPosition(drive_position, this->kTimeoutMs);
    ROS_INFO("SetPulseWidthPosition error code : %d for drive %d", error, drive->GetDeviceID());
  } while (error != ErrorCode::OKAY);
}

bool MarkhorHWInterfaceFlippers::hasResetOccurred()
{
  if (front_left_drive)
  {
    if (front_left_drive->HasResetOccurred() == true)
    {
      HasResetEventOccurred = true;
      return true;
    }
  }
  if (front_right_drive)
  {
    if (front_right_drive->HasResetOccurred() == true)
    {
      HasResetEventOccurred = true;
      return true;
    }
  }
  if (rear_left_drive)
  {
    if (rear_left_drive->HasResetOccurred() == true)
    {
      HasResetEventOccurred = true;
      return true;
    }
  }
  if (rear_right_drive)
  {
    if (rear_right_drive->HasResetOccurred() == true)
    {
      HasResetEventOccurred = true;
      return true;
    }
  }
  return false;
}
