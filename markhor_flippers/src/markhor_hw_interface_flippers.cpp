#include <ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h>
#include <unistd.h>
#include <string>
#include <std_msgs/Float64.h>

#include "markhor_hw_interface_flippers.hpp"

MarkhorHWInterfaceFlippers::MarkhorHWInterfaceFlippers()
{
  joint_names_.push_back("flipper_fl_j");
  joint_names_.push_back("flipper_fr_j");
  joint_names_.push_back("flipper_rl_j");
  joint_names_.push_back("flipper_rr_j");

  num_joints_ = joint_names_.size();

  // Status
  joint_position_.resize(num_joints_, 0.0);
  joint_velocity_.resize(num_joints_, 0.0);
  joint_effort_.resize(num_joints_, 0.0);

  // Command
  joint_position_command_.resize(num_joints_, 0.0);
  joint_velocity_command_.resize(num_joints_, 0.0);
  joint_effort_command_.resize(num_joints_, 0.0);

  setupRosControl();
  setupCtreDrive();

  nh_.getParam("/markhor/flippers/markhor_flippers_node/config_folder_location", config_folder_str_);
  nh_.getParam("/markhor/flippers/markhor_flippers_node/config_file_1", config_file_1_);
  nh_.getParam("/markhor/flippers/markhor_flippers_node/config_file_2", config_file_2_);

  fr_target_pub_ = nh_.advertise<std_msgs::Float64>("flipper_fr_position_target", 1000);
  fl_target_pub_ = nh_.advertise<std_msgs::Float64>("flipper_fl_position_target", 1000);
  rr_target_pub_ = nh_.advertise<std_msgs::Float64>("flipper_rr_position_target", 1000);
  rl_target_pub_ = nh_.advertise<std_msgs::Float64>("flipper_rl_position_target", 1000);

  fr_motor_current_pub_ = nh_.advertise<std_msgs::Float64>("flipper_fr_motor_current", 1000);
  fl_motor_current_pub_ = nh_.advertise<std_msgs::Float64>("flipper_fl_motor_current", 1000);
  rr_motor_current_pub_ = nh_.advertise<std_msgs::Float64>("flipper_rr_motor_current", 1000);
  rl_motor_current_pub_ = nh_.advertise<std_msgs::Float64>("flipper_rl_motor_current", 1000);

  fr_motor_bus_voltage_pub_ = nh_.advertise<std_msgs::Float64>("flipper_fr_bus_voltage", 1000);
  fl_motor_bus_voltage_pub_ = nh_.advertise<std_msgs::Float64>("flipper_fl_bus_voltage", 1000);
  rr_motor_bus_voltage_pub_ = nh_.advertise<std_msgs::Float64>("flipper_rr_bus_voltage", 1000);
  rl_motor_bus_voltage_pub_ = nh_.advertise<std_msgs::Float64>("flipper_rl_bus_voltage", 1000);

  loadDrivePosition();

  joint_position_command_[0] = 0;
  joint_position_command_[1] = 0;
  joint_position_command_[2] = 0;
  joint_position_command_[3] = 0;

  front_left_drive_->Set(ControlMode::Position, front_left_drive_base_position_);
  front_right_drive_->Set(ControlMode::Position, front_right_drive_base_position_);
  rear_left_drive_->Set(ControlMode::Position, rear_left_drive_base_position_);
  rear_right_drive_->Set(ControlMode::Position, rear_right_drive_base_position_);
}

void MarkhorHWInterfaceFlippers::setupRosControl()
{
  // connect and register the joint state and effort interfaces
  for (unsigned int joint_id = 0; joint_id < num_joints_; ++joint_id)
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
  current_limit_config.currentLimit = current_limit_;

  float kP, kI, kD = 0.0;
  nh_.getParam("/markhor/flippers/markhor_flippers_node/kP", kP);
  nh_.getParam("/markhor/flippers/markhor_flippers_node/kI", kI);
  nh_.getParam("/markhor/flippers/markhor_flippers_node/kD", kD);

  if(nh_.getParam("/markhor/flippers/markhor_flippers_node/allowable_closedloop_error", allowable_closedloop_error) == false)
  {
    ROS_WARN("Missing allowable_closedloop_error, assuming 100000");

  float fb_coeff = 1.0;
  if (!nh_.getParam("/markhor/flippers/markhor_flippers_node/fb_coeff", fb_coeff))
  {
    ROS_WARN("TRACKS : Missing argument fb_coeff, assuming 1");
  }

  if (nh_.getParam("/markhor/flippers/markhor_flippers_node/front_left", drive_fl_id_) == true)
  {
    front_left_drive_ = std::make_unique<TalonSRX>(drive_fl_id_);
    front_left_drive_->ConfigFactoryDefault();
    front_left_drive_->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, timeout_ms_);
    front_left_drive_->SetSensorPhase(true);
    front_left_drive_->ConfigSupplyCurrentLimit(current_limit_config);
    front_left_drive_->ConfigNominalOutputForward(0, timeout_ms_);
    front_left_drive_->ConfigNominalOutputReverse(0, timeout_ms_);
    front_left_drive_->ConfigAllowableClosedloopError(0, allowable_closedloop_error, timeout_ms_);
    front_left_drive_->ConfigSelectedFeedbackCoefficient(fb_coeff, 0, timeout_ms_);

    double front_left_peak_output_forward, front_left_peak_output_reverse = 0;
    nh_.getParam("/markhor/flippers/markhor_flippers_node/front_left_drive_peak_output_forward",
                 front_left_peak_output_forward);
    nh_.getParam("/markhor/flippers/markhor_flippers_node/front_left_drive_peak_output_reverse",
                 front_left_peak_output_reverse);

    front_left_drive_->ConfigPeakOutputForward(front_left_peak_output_forward, timeout_ms_);
    front_left_drive_->ConfigPeakOutputReverse(front_left_peak_output_reverse, timeout_ms_);

    front_left_drive_->SelectProfileSlot(0, 0);
    front_left_drive_->Config_kF(0, 0, timeout_ms_);
    front_left_drive_->Config_kP(0, kP, timeout_ms_);
    front_left_drive_->Config_kI(0, kI, timeout_ms_);
    front_left_drive_->Config_kD(0, kD, timeout_ms_);

    nh_.getParam("/markhor/flippers/markhor_flippers_node/front_left_drive_upper_limit", front_left_drive_upper_limit_);
    nh_.getParam("/markhor/flippers/markhor_flippers_node/front_left_drive_lower_limit", front_left_drive_lower_limit_);
  }
  if (nh_.getParam("/markhor/flippers/markhor_flippers_node/front_right", drive_fr_id_) == true)
  {
    front_right_drive_ = std::make_unique<TalonSRX>(drive_fr_id_);
    front_right_drive_->ConfigFactoryDefault();
    front_right_drive_->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, timeout_ms_);
    front_right_drive_->SetSensorPhase(true);
    front_right_drive_->ConfigSupplyCurrentLimit(current_limit_config);
    front_right_drive_->ConfigNominalOutputForward(0, timeout_ms_);
    front_right_drive_->ConfigNominalOutputReverse(0, timeout_ms_);
    front_right_drive_->ConfigAllowableClosedloopError(0, allowable_closedloop_error, timeout_ms_);
    front_right_drive_->ConfigSelectedFeedbackCoefficient(fb_coeff, 0, timeout_ms_);


    double front_right_peak_output_forward, front_right_peak_output_reverse = 0;
    nh_.getParam("/markhor/flippers/markhor_flippers_node/front_right_drive_peak_output_forward",
                 front_right_peak_output_forward);
    nh_.getParam("/markhor/flippers/markhor_flippers_node/front_right_drive_peak_output_reverse",
                 front_right_peak_output_reverse);

    front_right_drive_->ConfigPeakOutputForward(front_right_peak_output_forward, timeout_ms_);
    front_right_drive_->ConfigPeakOutputReverse(front_right_peak_output_reverse, timeout_ms_);
    rear_right_drive_->ConfigAllowableClosedloopError(0, 100, timeout_ms_);

    front_right_drive_->SelectProfileSlot(0, 0);
    front_right_drive_->Config_kF(0, 0, timeout_ms_);
    front_right_drive_->Config_kP(0, kP, timeout_ms_);
    front_right_drive_->Config_kI(0, kI, timeout_ms_);
    front_right_drive_->Config_kD(0, kD, timeout_ms_);

    nh_.getParam("/markhor/flippers/markhor_flippers_node/front_right_drive_upper_limit",
                 front_right_drive_upper_limit_);
    nh_.getParam("/markhor/flippers/markhor_flippers_node/front_right_drive_lower_limit",
                 front_right_drive_lower_limit_);
  }
  if (nh_.getParam("/markhor/flippers/markhor_flippers_node/rear_left", drive_rl_id_) == true)
  {
    rear_left_drive_ = std::make_unique<TalonSRX>(drive_rl_id_);
    rear_left_drive_->ConfigFactoryDefault();
    rear_left_drive_->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, timeout_ms_);
    rear_left_drive_->SetSensorPhase(true);
    rear_left_drive_->ConfigSupplyCurrentLimit(current_limit_config);
    rear_left_drive_->ConfigNominalOutputForward(0, timeout_ms_);
    rear_left_drive_->ConfigNominalOutputReverse(0, timeout_ms_);
    rear_left_drive_->ConfigSelectedFeedbackCoefficient(fb_coeff, 0, timeout_ms_);

    double rear_left_peak_output_forward, rear_left_peak_output_reverse = 0;
    nh_.getParam("/markhor/flippers/markhor_flippers_node/rear_left_drive_peak_output_forward",
                 rear_left_peak_output_forward);
    nh_.getParam("/markhor/flippers/markhor_flippers_node/rear_left_drive_peak_output_reverse",
                 rear_left_peak_output_reverse);

    rear_left_drive_->ConfigPeakOutputForward(rear_left_peak_output_forward, timeout_ms_);
    rear_left_drive_->ConfigPeakOutputReverse(rear_left_peak_output_reverse, timeout_ms_);
    rear_left_drive_->ConfigAllowableClosedloopError(0, allowable_closedloop_error, timeout_ms_);

    rear_left_drive_->SelectProfileSlot(0, 0);
    rear_left_drive_->Config_kF(0, 0, timeout_ms_);
    rear_left_drive_->Config_kP(0, kP, timeout_ms_);
    rear_left_drive_->Config_kI(0, kI, timeout_ms_);
    rear_left_drive_->Config_kD(0, kD, timeout_ms_);

    nh_.getParam("/markhor/flippers/markhor_flippers_node/rear_left_drive_upper_limit", rear_left_drive_upper_limit_);
    nh_.getParam("/markhor/flippers/markhor_flippers_node/rear_left_drive_lower_limit", rear_left_drive_lower_limit_);
  }
  if (nh_.getParam("/markhor/flippers/markhor_flippers_node/rear_right", drive_rr_id_) == true)
  {
    rear_right_drive_ = std::make_unique<TalonSRX>(drive_rr_id_);
    rear_right_drive_->ConfigFactoryDefault();
    rear_right_drive_->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, timeout_ms_);
    rear_right_drive_->SetSensorPhase(true);
    rear_right_drive_->ConfigSupplyCurrentLimit(current_limit_config);
    rear_right_drive_->ConfigNominalOutputForward(0, timeout_ms_);
    rear_right_drive_->ConfigNominalOutputReverse(0, timeout_ms_);
    rear_right_drive_->ConfigSelectedFeedbackCoefficient(fb_coeff, 0, timeout_ms_);

    double rear_right_peak_output_forward, rear_right_peak_output_reverse = 0;
    nh_.getParam("/markhor/flippers/markhor_flippers_node/rear_right_drive_peak_output_forward",
                 rear_right_peak_output_forward);
    nh_.getParam("/markhor/flippers/markhor_flippers_node/rear_right_drive_peak_output_reverse",
                 rear_right_peak_output_reverse);

    rear_right_drive_->ConfigPeakOutputForward(rear_right_peak_output_forward, timeout_ms_);
    rear_right_drive_->ConfigPeakOutputReverse(rear_right_peak_output_reverse, timeout_ms_);
    rear_right_drive_->ConfigAllowableClosedloopError(0, allowable_closedloop_error, timeout_ms_);

    rear_right_drive_->SelectProfileSlot(0, 0);
    rear_right_drive_->Config_kF(0, 0, timeout_ms_);
    rear_right_drive_->Config_kP(0, kP, timeout_ms_);
    rear_right_drive_->Config_kI(0, kI, timeout_ms_);
    rear_right_drive_->Config_kD(0, kD, timeout_ms_);

    nh_.getParam("/markhor/flippers/markhor_flippers_node/rear_right_drive_upper_limit", rear_right_drive_upper_limit_);
    nh_.getParam("/markhor/flippers/markhor_flippers_node/rear_right_drive_lower_limit", rear_right_drive_lower_limit_);
  }
}

void MarkhorHWInterfaceFlippers::write()
{
  // Write to drive

  if (hasResetOccurred() == true)
  {
    loadDrivePosition();
    accumulator_fl_ = 0;
    accumulator_fr_ = 0;
    accumulator_rl_ = 0;
    accumulator_rr_ = 0;
  }

  ctre::phoenix::unmanaged::FeedEnable(100);

  /*
  The lines :
    Lower limit :
    front_left_drive_lower_limit_ <= front_left_drive_base_position + accumulator_fl + joint_position_command_[0]

    And

    Upper limit :
    front_left_drive_base_position + accumulator_fl + joint_position_command_[0] < front_left_drive_upper_limit_

  in each if condition for each drive, represent if the next position value we want to apply to the flipper is going to
  be above or under the limit of the flipper.
 */

  // For debug purposes use: printDriveInfo(<drive>);

  if (front_left_drive_lower_limit_ <= front_left_drive_base_position_ + accumulator_fl_ + joint_position_command_[0] &&
      front_left_drive_base_position_ + accumulator_fl_ + joint_position_command_[0] < front_left_drive_upper_limit_)
  {
    accumulator_fl_ += joint_position_command_[0];
    target_fl_ = front_left_drive_base_position_ + accumulator_fl_;
    ROS_INFO_THROTTLE(1, "target = [%f]", target_fl_);
    front_left_drive_->Set(ControlMode::Position, target_fl_);
  }

  if (front_right_drive_lower_limit_ <=
          front_right_drive_base_position_ + accumulator_fr_ + joint_position_command_[1] &&
      front_right_drive_base_position_ + accumulator_fr_ + joint_position_command_[1] < front_right_drive_upper_limit_)
  {
    accumulator_fr_ += joint_position_command_[1];
    target_fr_ = front_right_drive_base_position_ + accumulator_fr_;
    ROS_INFO_THROTTLE(1, "target = [%f]", target_fr_);
    front_right_drive_->Set(ControlMode::Position, target_fr_);
  }

  if (rear_left_drive_lower_limit_ <= rear_left_drive_base_position_ + accumulator_rl_ + joint_position_command_[2] &&
      rear_left_drive_base_position_ + accumulator_rl_ + joint_position_command_[2] < rear_left_drive_upper_limit_)
  {
    accumulator_rl_ += joint_position_command_[2];
    target_rl_ = rear_left_drive_base_position_ + accumulator_rl_;
    ROS_INFO_THROTTLE(1, "target = [%f]", target_rl_);
    rear_left_drive_->Set(ControlMode::Position, target_rl_);
  }

  if (rear_right_drive_lower_limit_ <= rear_right_drive_base_position_ + accumulator_rr_ + joint_position_command_[3] &&
      rear_right_drive_base_position_ + accumulator_rr_ + joint_position_command_[3] < rear_right_drive_upper_limit_)
  {
    accumulator_rr_ += joint_position_command_[3];
    target_rr_ = rear_right_drive_base_position_ + accumulator_rr_;
    ROS_INFO_THROTTLE(1, "target = [%f]", target_rr_);
    rear_right_drive_->Set(ControlMode::Position, target_rr_);
  }
  saveDrivePosition();
}

void MarkhorHWInterfaceFlippers::read()
{
  // Read from the motor API, going to read from the TalonSRX objects

  publishTarget();
  publishMotorCurrent();
  publishMotorBusVoltage();

  joint_position_[0] = front_left_drive_->GetSensorCollection().GetPulseWidthPosition();
  joint_position_[1] = front_right_drive_->GetSensorCollection().GetPulseWidthPosition();
  joint_position_[2] = rear_left_drive_->GetSensorCollection().GetPulseWidthPosition();
  joint_position_[3] = rear_right_drive_->GetSensorCollection().GetPulseWidthPosition();
}

void MarkhorHWInterfaceFlippers::printDriveInfo(std::unique_ptr<TalonSRX>& drive)
{
  ROS_INFO_THROTTLE(1, "-------------");
  if (drive->GetDeviceID() == drive_fl_id_)
  {
    ROS_INFO_THROTTLE(1, "position FL command : %f", joint_position_command_[0]);
    ROS_INFO_THROTTLE(1, "FL lower limit : %f", front_left_drive_lower_limit_);
    ROS_INFO_THROTTLE(1, "FL upper limit : %f", front_left_drive_upper_limit_);
    ROS_INFO_THROTTLE(1, "front_left_drive_base_position : %f", front_left_drive_base_position_);
    ROS_INFO_THROTTLE(1, "accumulator_fl : %f", accumulator_fl_);
    ROS_INFO_THROTTLE(1, "Target : %f", target_fl_);
    ROS_INFO_THROTTLE(1, "Drive %d : output voltage : %f", front_left_drive_->GetDeviceID(),
                      front_left_drive_->GetMotorOutputVoltage());
  }
  else if (drive->GetDeviceID() == drive_fr_id_)
  {
    ROS_INFO_THROTTLE(1, "position FR command : %f", joint_position_command_[1]);
    ROS_INFO_THROTTLE(1, "FR lower limit : %f", front_right_drive_lower_limit_);
    ROS_INFO_THROTTLE(1, "FR upper limit : %f", front_right_drive_upper_limit_);
    ROS_INFO_THROTTLE(1, "front_right_drive_base_position : %f", front_right_drive_base_position_);
    ROS_INFO_THROTTLE(1, "accumulator_fr : %f", accumulator_fr_);
    ROS_INFO_THROTTLE(1, "Target : %f", target_fr_);
    ROS_INFO_THROTTLE(1, "Drive %d : output voltage : %f", front_right_drive_->GetDeviceID(),
                      front_right_drive_->GetMotorOutputVoltage());
  }
  else if (drive->GetDeviceID() == drive_rl_id_)
  {
    ROS_INFO_THROTTLE(1, "position RL command : %f", joint_position_command_[2]);
    ROS_INFO_THROTTLE(1, "RL lower limit : %f", rear_left_drive_lower_limit_);
    ROS_INFO_THROTTLE(1, "RL upper limit : %f", rear_left_drive_upper_limit_);
    ROS_INFO_THROTTLE(1, "rear_left_drive_base_position : %f", rear_left_drive_base_position_);
    ROS_INFO_THROTTLE(1, "accumulator_rl : %f", accumulator_rl_);
    ROS_INFO_THROTTLE(1, "Target : %f", target_rl_);
    ROS_INFO_THROTTLE(1, "Drive %d : output voltage : %f", rear_left_drive_->GetDeviceID(),
                      rear_left_drive_->GetMotorOutputVoltage());
  }
  else if (drive->GetDeviceID() == drive_rr_id_)
  {
    ROS_INFO_THROTTLE(1, "position RR command : %f", joint_position_command_[3]);
    ROS_INFO_THROTTLE(1, "RR lower limit : %f", rear_right_drive_lower_limit_);
    ROS_INFO_THROTTLE(1, "RR upper limit : %f", rear_right_drive_upper_limit_);
    ROS_INFO_THROTTLE(1, "rear_right_drive_base_position : %f", rear_right_drive_base_position_);
    ROS_INFO_THROTTLE(1, "accumulator_rr : %f", accumulator_rl_);
    ROS_INFO_THROTTLE(1, "Target : %f", target_rr_);
    ROS_INFO_THROTTLE(1, "Drive %d : output voltage : %f", rear_right_drive_->GetDeviceID(),
                      rear_right_drive_->GetMotorOutputVoltage());
  }
  else
  {
    ROS_INFO_THROTTLE(1, "Device ID is not found");
    return;
  }

  ROS_INFO_THROTTLE(1, "GetPulseWidthPosition %d", drive->GetSensorCollection().GetPulseWidthPosition());
  ROS_INFO_THROTTLE(1, "GetClosedLoopError %d", drive->GetClosedLoopError(0));
  ROS_INFO_THROTTLE(1, "GetClosedLoopTarget %f", drive->GetClosedLoopTarget(0));
}

void MarkhorHWInterfaceFlippers::saveDrivePosition()
{
  // TODO : Only write if the value is different

  if (alternating_value_ == true)
  {
    alternating_value_ = false;
    writeDrivePositionToFile(config_file_1_);
  }
  else
  {
    alternating_value_ = true;
    writeDrivePositionToFile(config_file_2_);
  }
}

void MarkhorHWInterfaceFlippers::writeDrivePositionToFile(std::string config_file_name)
{
  drive_config_file_.open((config_folder_str_ + config_file_name).c_str(), std::fstream::out | std::fstream::trunc);
  if (!drive_config_file_)
  {
    ROS_ERROR("%s - Can't open file %s", __FUNCTION__, config_file_name.c_str());
    ros::shutdown();
    return;
  }

  if (front_left_drive_)
  {
    drive_config_file_ << front_left_drive_->GetDeviceID() << ":" << getEncoderPosition(front_left_drive_) << std::endl;
  }
  if (front_right_drive_)
  {
    drive_config_file_ << front_right_drive_->GetDeviceID() << ":" << getEncoderPosition(front_right_drive_)
                       << std::endl;
  }
  if (rear_left_drive_)
  {
    drive_config_file_ << rear_left_drive_->GetDeviceID() << ":" << getEncoderPosition(rear_left_drive_) << std::endl;
  }
  if (rear_right_drive_)
  {
    drive_config_file_ << rear_right_drive_->GetDeviceID() << ":" << getEncoderPosition(rear_right_drive_)
                       << std::flush;
  }
  drive_config_file_.rdbuf()->pubsync();
  drive_config_file_.close();
}

int MarkhorHWInterfaceFlippers::getEncoderPosition(std::unique_ptr<TalonSRX>& drive)
{
  int pulse_width_position = drive->GetSensorCollection().GetPulseWidthPosition();
  if (drive->GetDeviceID() == drive_fl_id_)
  {
    if (pulse_width_position < -1 * front_left_drive_upper_limit_)
    {
      return -1 * front_left_drive_upper_limit_;
    }
    else if (pulse_width_position > -1 * front_left_drive_lower_limit_)
    {
      return -1 * front_left_drive_lower_limit_;
    }
    else
    {
      return pulse_width_position;
    }
  }
  else if (drive->GetDeviceID() == drive_fr_id_)
  {
    if (pulse_width_position < -1 * front_right_drive_upper_limit_)
    {
      return -1 * front_right_drive_upper_limit_;
    }
    else if (pulse_width_position > -1 * front_right_drive_lower_limit_)
    {
      return -1 * front_right_drive_lower_limit_;
    }
    else
    {
      return pulse_width_position;
    }
  }
  else if (drive->GetDeviceID() == drive_rl_id_)
  {
    if (pulse_width_position < -1 * rear_left_drive_upper_limit_)
    {
      return -1 * rear_left_drive_upper_limit_;
    }
    else if (pulse_width_position > -1 * rear_left_drive_lower_limit_)
    {
      return -1 * rear_left_drive_lower_limit_;
    }
    else
    {
      return pulse_width_position;
    }
  }
  else if (drive->GetDeviceID() == drive_rr_id_)
  {
    if (pulse_width_position < -1 * rear_right_drive_upper_limit_)
    {
      return -1 * rear_right_drive_upper_limit_;
    }
    else if (pulse_width_position > -1 * rear_right_drive_lower_limit_)
    {
      return -1 * rear_right_drive_lower_limit_;
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
  drive_config_file_1.open((config_folder_str_ + config_file_1_).c_str(), std::fstream::in);
  drive_config_file_2.open((config_folder_str_ + config_file_2_).c_str(), std::fstream::in);
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
    readDrivePositionFromFile(config_file_1_, drive_config_file_1);
  }
  if (is_drives_config_file_2_empty == false)
  {
    readDrivePositionFromFile(config_file_2_, drive_config_file_2);
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

  if (drive_id == drive_fl_id_)
  {
    applyDrivePosition(front_left_drive_, drive_position);
  }
  if (drive_id == drive_fr_id_)
  {
    applyDrivePosition(front_right_drive_, drive_position);
  }
  if (drive_id == drive_rl_id_)
  {
    applyDrivePosition(rear_left_drive_, drive_position);
  }
  if (drive_id == drive_rr_id_)
  {
    applyDrivePosition(rear_right_drive_, drive_position);
  }

  has_reset_event_occured_ = false;
}

void MarkhorHWInterfaceFlippers::applyDrivePosition(std::unique_ptr<TalonSRX>& drive, float drive_position)
{
  int error;
  if (has_reset_event_occured_ == false)
  {
    if (drive->GetDeviceID() == drive_fl_id_)
    {
      front_left_drive_base_position_ = -1 * drive_position;
    }
    else if (drive->GetDeviceID() == drive_fr_id_)
    {
      front_right_drive_base_position_ = -1 * drive_position;
    }
    else if (drive->GetDeviceID() == drive_rl_id_)
    {
      rear_left_drive_base_position_ = -1 * drive_position;
    }
    else if (drive->GetDeviceID() == drive_rr_id_)
    {
      rear_right_drive_base_position_ = -1 * drive_position;
    }
  }
  do
  {
    error = drive->SetSelectedSensorPosition(-1 * drive_position, 0, timeout_ms_);
    ROS_INFO_THROTTLE(1, "SetSelectedSensorPosition error code : %d for drive %d", error, drive->GetDeviceID());
  } while (error != ErrorCode::OKAY);
}

bool MarkhorHWInterfaceFlippers::hasResetOccurred()
{
  if (front_left_drive_)
  {
    if (front_left_drive_->HasResetOccurred() == true)
    {
      has_reset_event_occured_ = true;
      return true;
    }
  }
  if (front_right_drive_)
  {
    if (front_right_drive_->HasResetOccurred() == true)
    {
      has_reset_event_occured_ = true;
      return true;
    }
  }
  if (rear_left_drive_)
  {
    if (rear_left_drive_->HasResetOccurred() == true)
    {
      has_reset_event_occured_ = true;
      return true;
    }
  }
  if (rear_right_drive_)
  {
    if (rear_right_drive_->HasResetOccurred() == true)
    {
      has_reset_event_occured_ = true;
      return true;
    }
  }
  return false;
}

bool MarkhorHWInterfaceFlippers::publishTarget()
{
  fl_target_pub_.publish(createMessage(target_fl_));
  fr_target_pub_.publish(createMessage(target_fr_));
  rl_target_pub_.publish(createMessage(target_rl_));
  rr_target_pub_.publish(createMessage(target_rr_));
  return true;
}

bool MarkhorHWInterfaceFlippers::publishMotorCurrent()
{
  fl_motor_current_pub_.publish(createMessage(front_left_drive_->GetOutputCurrent()));
  fr_motor_current_pub_.publish(createMessage(front_right_drive_->GetOutputCurrent()));
  rl_motor_current_pub_.publish(createMessage(rear_left_drive_->GetOutputCurrent()));
  rr_motor_current_pub_.publish(createMessage(rear_right_drive_->GetOutputCurrent()));
  return true;
}

bool MarkhorHWInterfaceFlippers::publishMotorBusVoltage()
{
  fl_motor_bus_voltage_pub_.publish(createMessage(front_left_drive_->GetBusVoltage()));
  fr_motor_bus_voltage_pub_.publish(createMessage(front_right_drive_->GetBusVoltage()));
  rl_motor_bus_voltage_pub_.publish(createMessage(rear_left_drive_->GetBusVoltage()));
  rr_motor_bus_voltage_pub_.publish(createMessage(rear_right_drive_->GetBusVoltage()));
  return true;
}

std_msgs::Float64 MarkhorHWInterfaceFlippers::createMessage(float value)
{
  std_msgs::Float64 msg;
  msg.data = value;
  return msg;
}
