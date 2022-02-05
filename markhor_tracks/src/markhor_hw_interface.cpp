#include <markhor_hw_interface.hpp>

std::string drives_name[] = { "flipper_fl_motor_j", "flipper_fr_motor_j", "flipper_rl_motor_j", "flipper_rr_motor_j" };

MarkhorHWInterface::MarkhorHWInterface()
  : running_(true)
  , private_nh("~")
  , start_srv_(nh.advertiseService("start", &MarkhorHWInterface::start_callback, this))
  , stop_srv_(nh.advertiseService("stop", &MarkhorHWInterface::start_callback, this))
{
  std::fill(pos, pos + NUM_JOINTS, 0.0);
  std::fill(vel, vel + NUM_JOINTS, 0.0);
  std::fill(eff, eff + NUM_JOINTS, 0.0);
  std::fill(cmd, cmd + NUM_JOINTS, 0.0);

  setupRosControl();
  setupCTREDrive();
  setupPublisher();
}

void MarkhorHWInterface::setupPublisher()
{
  // Initialize publishers and subscribers
  front_left_track_vel_pub_ = nh.advertise<std_msgs::Float32>("front_left_track_vel", 1);
  rear_left_track_vel_pub_ = nh.advertise<std_msgs::Float32>("rear_right_track_vel", 1);
  front_right_track_vel_pub_ = nh.advertise<std_msgs::Float32>("front_right_track_vel", 1);
  rear_right_track_vel_pub_ = nh.advertise<std_msgs::Float32>("rear_left_track_vel", 1);
}

void MarkhorHWInterface::setupRosControl()
{
  // connect and register the joint state and velocity interfaces
  for (unsigned int i = 0; i < NUM_JOINTS; ++i)
  {
    hardware_interface::JointStateHandle state_handle(drives_name[i], &pos[i], &vel[i], &eff[i]);
    jnt_state_interface.registerHandle(state_handle);

    hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(drives_name[i]), &cmd[i]);
    jnt_vel_interface.registerHandle(vel_handle);
  }

  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_vel_interface);
}

void MarkhorHWInterface::setupCTREDrive()
{
  std::string interface = "can0";
  ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

  int drive_fl_id, drive_fr_id, drive_rl_id, drive_rr_id = 0;

  SupplyCurrentLimitConfiguration current_limit_config;
  current_limit_config.enable = true;
  current_limit_config.currentLimit = 30;

  if (nh.getParam("/markhor/tracks/markhor_tracks_node/front_left", drive_fl_id) == true)
  {
    front_left_drive = std::make_unique<TalonSRX>(drive_fl_id);
    front_left_drive->SetNeutralMode(NeutralMode::Coast);
    front_left_drive->SetInverted(true);
    front_left_drive->ConfigFactoryDefault();
    front_left_drive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative , 0, timeout_ms_);
    front_left_drive->SetSensorPhase(false);
    front_left_drive->ConfigSupplyCurrentLimit(current_limit_config);
    front_left_drive->ConfigNominalOutputForward(0, timeout_ms_);
    front_left_drive->ConfigNominalOutputReverse(0, timeout_ms_);
    front_left_drive->ConfigAllowableClosedloopError(0, 0, timeout_ms_);
    front_left_drive->SelectProfileSlot(0, 0);
    front_left_drive->Config_kF(0, 0, timeout_ms_);
    front_left_drive->Config_kP(0, tracks_kp, timeout_ms_);
    front_left_drive->Config_kI(0, tracks_ki, timeout_ms_);
    front_left_drive->Config_kD(0, 0, timeout_ms_);
    front_left_drive->ConfigMaxIntegralAccumulator(0, integral_max, timeout_ms_);
    front_left_drive->Config_IntegralZone(0, integral_zone, timeout_ms_);

    ctre::phoenix::unmanaged::FeedEnable(timeout_ms_);
    front_left_drive->Set(ControlMode::Velocity, 0);
  }
  if (nh.getParam("/markhor/tracks/markhor_tracks_node/rear_left", drive_rl_id) == true)
  {
    rear_left_drive = std::make_unique<TalonSRX>(drive_rl_id);
    rear_left_drive->SetNeutralMode(NeutralMode::Coast);
    rear_left_drive->ConfigFactoryDefault();
    rear_left_drive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative , 0, timeout_ms_);
    rear_left_drive->SetSensorPhase(true);
    rear_left_drive->ConfigSupplyCurrentLimit(current_limit_config);
    rear_left_drive->ConfigNominalOutputForward(0, timeout_ms_);
    rear_left_drive->ConfigNominalOutputReverse(0, timeout_ms_);
    rear_left_drive->ConfigAllowableClosedloopError(0, 0, timeout_ms_);
    rear_left_drive->SelectProfileSlot(0, 0);
    rear_left_drive->Config_kF(0, 0, timeout_ms_);
    rear_left_drive->Config_kP(0, tracks_kp, timeout_ms_);
    rear_left_drive->Config_kI(0, tracks_ki, timeout_ms_);
    rear_left_drive->Config_kD(0, 0, timeout_ms_);
    rear_left_drive->ConfigMaxIntegralAccumulator(0, integral_max, timeout_ms_);
    rear_left_drive->Config_IntegralZone(0, integral_zone, timeout_ms_);

    ctre::phoenix::unmanaged::FeedEnable(timeout_ms_);
    rear_left_drive->Set(ControlMode::Velocity, 0);
  }
  if (nh.getParam("/markhor/tracks/markhor_tracks_node/front_right", drive_fr_id) == true)
  {
    front_right_drive = std::make_unique<TalonSRX>(drive_fr_id);
    front_right_drive->SetNeutralMode(NeutralMode::Coast);
    front_right_drive->SetInverted(true);
    front_right_drive->ConfigFactoryDefault();
    front_right_drive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative , 0, timeout_ms_);
    front_right_drive->SetSensorPhase(true);
    front_right_drive->ConfigSupplyCurrentLimit(current_limit_config);
    front_right_drive->ConfigNominalOutputForward(0, timeout_ms_);
    front_right_drive->ConfigNominalOutputReverse(0, timeout_ms_);
    front_right_drive->ConfigAllowableClosedloopError(0, 0, timeout_ms_);
    front_right_drive->SelectProfileSlot(0, 0);
    front_right_drive->Config_kF(0, 0, timeout_ms_);
    front_right_drive->Config_kP(0, tracks_kp, timeout_ms_);
    front_right_drive->Config_kI(0, tracks_ki, timeout_ms_);
    front_right_drive->Config_kD(0, 0, timeout_ms_);
    front_right_drive->ConfigMaxIntegralAccumulator(0, integral_max, timeout_ms_);
    front_right_drive->Config_IntegralZone(0, integral_zone, timeout_ms_);

    ctre::phoenix::unmanaged::FeedEnable(timeout_ms_);
    front_right_drive->Set(ControlMode::Velocity, 0);
  }
  if (nh.getParam("/markhor/tracks/markhor_tracks_node/rear_right", drive_rr_id) == true)
  {
    rear_right_drive = std::make_unique<TalonSRX>(drive_rr_id);
    rear_right_drive->SetNeutralMode(NeutralMode::Coast);
    rear_right_drive->ConfigFactoryDefault();
    rear_right_drive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative , 0, timeout_ms_);
    rear_right_drive->SetSensorPhase(false);
    rear_right_drive->ConfigSupplyCurrentLimit(current_limit_config);
    rear_right_drive->ConfigNominalOutputForward(0, timeout_ms_);
    rear_right_drive->ConfigNominalOutputReverse(0, timeout_ms_);
    rear_right_drive->ConfigAllowableClosedloopError(0, 0, timeout_ms_);
    rear_right_drive->SelectProfileSlot(0, 0);
    rear_right_drive->Config_kF(0, 0, timeout_ms_);
    rear_right_drive->Config_kP(0, tracks_kp, timeout_ms_);
    rear_right_drive->Config_kI(0, tracks_ki, timeout_ms_);
    rear_right_drive->Config_kD(0, 0, timeout_ms_);
    rear_right_drive->ConfigMaxIntegralAccumulator(0, integral_max, timeout_ms_);
    rear_right_drive->Config_IntegralZone(0, integral_zone, timeout_ms_);

    rear_right_drive->ConfigSelectedFeedbackCoefficient(1.0/3.0, 0, timeout_ms_);//HOTFIX for the encoder that returned 3x more steps than the others

    ctre::phoenix::unmanaged::FeedEnable(timeout_ms_);
    rear_right_drive->Set(ControlMode::Velocity, 0);
  }
}

void MarkhorHWInterface::write()
{
  double diff_ang_speed_front_left = cmd[0] * 125 * 4;
  double diff_ang_speed_front_right = cmd[1] * 125 * 4;
  double diff_ang_speed_rear_left = cmd[2] * 125 * 4;
  double diff_ang_speed_rear_right = cmd[3] * 125 * 4;

  ROS_DEBUG_THROTTLE(1,"\n\rCommand :");
  ROS_DEBUG_THROTTLE(1,"FWD_L: %lf, FWD_R: %lf", diff_ang_speed_front_right, diff_ang_speed_front_left);
  ROS_DEBUG_THROTTLE(1,"AFT_L: %lf, AFT_R: %lf", diff_ang_speed_rear_right, diff_ang_speed_rear_left);
  ROS_DEBUG_THROTTLE(1,"Encoder Velocity:");
  ROS_DEBUG_THROTTLE(1,"FWD_L: %d, FWD_R: %d", front_left_drive->GetSensorCollection().GetQuadratureVelocity(),front_right_drive->GetSensorCollection().GetQuadratureVelocity());
  ROS_DEBUG_THROTTLE(1,"AFT_L: %d, AFT_R: %d", rear_left_drive->GetSensorCollection().GetQuadratureVelocity(),rear_right_drive->GetSensorCollection().GetQuadratureVelocity());

  ctre::phoenix::unmanaged::FeedEnable(timeout_ms_);

  // Set data
  front_left_track_vel_msg.data = diff_ang_speed_front_left;
  front_right_track_vel_msg.data = diff_ang_speed_front_right;
  rear_left_track_vel_msg.data = diff_ang_speed_rear_left;
  rear_right_track_vel_msg.data = diff_ang_speed_rear_right;


  // Write to drive
  front_left_drive->Set(ControlMode::Velocity, front_left_track_vel_msg.data);
  front_right_drive->Set(ControlMode::Velocity, front_right_track_vel_msg.data);
  rear_left_drive->Set(ControlMode::Velocity, rear_left_track_vel_msg.data);
  rear_right_drive->Set(ControlMode::Velocity, rear_right_track_vel_msg.data);

  // Publish data
  front_left_track_vel_pub_.publish(front_left_track_vel_msg);
  front_right_track_vel_pub_.publish(front_right_track_vel_msg);
  rear_left_track_vel_pub_.publish(rear_left_track_vel_msg);
  rear_right_track_vel_pub_.publish(rear_right_track_vel_msg);
}

void MarkhorHWInterface::read(const ros::Duration& period)
{
    // Read from the motor API, going to read from the TalonSRX objects
    //ROS_INFO("Vel: %d, %d", rear_right_drive->GetSensorCollection().GetQuadratureVelocity(),rear_left_drive->GetSensorCollection().GetQuadratureVelocity());
}

ros::Time MarkhorHWInterface::get_time()
{
  prev_update_time = curr_update_time;
  curr_update_time = ros::Time::now();
  return curr_update_time;
}

ros::Duration MarkhorHWInterface::get_period()
{
  return curr_update_time - prev_update_time;
}

bool MarkhorHWInterface::start_callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  running_ = true;
  return true;
}

bool MarkhorHWInterface::stop_callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  running_ = false;
  return true;
}