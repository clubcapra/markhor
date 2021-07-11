#include <markhor_hw_interface_flipper.hpp>

std::string drives_name[NUM_JOINTS] = { "flipper_fl_j"};

MarkhorHWInterfaceFlipper::MarkhorHWInterfaceFlipper()
  : running_(true)
  , private_nh("~")
  , start_srv_(nh.advertiseService("start", &MarkhorHWInterfaceFlipper::start_callback, this))
  , stop_srv_(nh.advertiseService("stop", &MarkhorHWInterfaceFlipper::start_callback, this))
{
  private_nh.param<double>("max_speed", max_speed, 1.0);

  std::fill(pos, pos + NUM_JOINTS, 0.0);
  std::fill(vel, vel + NUM_JOINTS, 0.0);
  std::fill(eff, eff + NUM_JOINTS, 0.0);
  std::fill(cmd, cmd + NUM_JOINTS, 0.0);

  setupRosControl();
  setupCTREDrive();
  setupPublisher();
}

void MarkhorHWInterfaceFlipper::setupPublisher()
{
  // Initialize publishers and subscribers
  // front_left_track_vel_pub_ = nh.advertise<std_msgs::Float32>("front_left_track_vel", 1);
}

void MarkhorHWInterfaceFlipper::setupRosControl()
{
  // connect and register the joint state and velocity interfaces
  for (unsigned int i = 0; i < NUM_JOINTS; ++i)
  {
    hardware_interface::JointStateHandle state_handle(drives_name[i], &pos[i], &vel[i], &eff[i]);
    jnt_state_interface.registerHandle(state_handle);

    hardware_interface::JointHandle eff_handle(jnt_state_interface.getHandle(drives_name[i]), &cmd[i]);
    jnt_eff_interface.registerHandle(eff_handle);
  }

  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_eff_interface);
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
  // if (nh.getParam("/markhor/markhor_base_node/rear_left", drive_rl_id) == true)
  // {
  //   rear_left_drive = std::make_unique<TalonSRX>(drive_rl_id);
  // }
  // if (nh.getParam("/markhor/markhor_base_node/front_right", drive_fr_id) == true)
  // {
  //   front_right_drive = std::make_unique<TalonSRX>(drive_fr_id);
  // }
  // if (nh.getParam("/markhor/markhor_base_node/rear_right", drive_rr_id) == true)
  // {
  //   rear_right_drive = std::make_unique<TalonSRX>(drive_rr_id);
  // }
}

void MarkhorHWInterfaceFlipper::write()
{

  ROS_INFO("HWI FLIPPER WRITE");
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

void MarkhorHWInterfaceFlipper::read(const ros::Duration& period)
{
  // Read from the motor API, going to read from the TalonSRX objects
  ROS_INFO("HWI FLIPPER READs");
}

ros::Time MarkhorHWInterfaceFlipper::get_time()
{
  prev_update_time = curr_update_time;
  curr_update_time = ros::Time::now();
  return curr_update_time;
}

ros::Duration MarkhorHWInterfaceFlipper::get_period()
{
  return curr_update_time - prev_update_time;
}

bool MarkhorHWInterfaceFlipper::start_callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  running_ = true;
  return true;
}

bool MarkhorHWInterfaceFlipper::stop_callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  running_ = false;
  return true;
}

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