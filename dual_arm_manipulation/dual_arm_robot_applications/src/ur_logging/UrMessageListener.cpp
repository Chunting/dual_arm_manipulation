#include "ur_logging/UrMessageListener.h"

// UR_Message_Listener::UR_Message_Listener(ros::NodeHandle& nh, std::string ur_namespace) : nh_(nh),
// ur_namespace_(ur_namespace){
UR_Message_Listener::UR_Message_Listener(ros::NodeHandle &nh, std::string ur_namespace, std::string folder_name,
                                         double frequency)
  : nh_(nh), ur_namespace_(ur_namespace), folder_name_(folder_name), loop_rate_(frequency)
{
  if (ur_namespace_.size() > 0)
  {
    ur_prefix_ = ur_namespace_ + "_";
  }
  topic_cartesian_state_ = ur_namespace_ + "/ur5_cartesian_velocity_controller/ee_state";
  topic_cartesian_cmd_ = ur_namespace_ + "/ur5_cartesian_velocity_controller/command_cart_vel";
  topic_external_wrench = ur_namespace_ + "/robotiq_ft_wrench";
  topic_joint_state_ = "/joint_states";
  topic_joint_cmd_ = ur_namespace + "/ur_driver/joint_speed";

  delimiter_ = ',';

  sub_cartesian_state_ =
      nh_.subscribe(topic_cartesian_state_, 100, &UR_Message_Listener::cartesian_state_callback, this);
  sub_cartesian_cmd_ = nh_.subscribe(topic_cartesian_cmd_, 100, &UR_Message_Listener::cartesian_cmd_callback, this);
  sub_wrench_external_ = nh_.subscribe(topic_external_wrench, 100, &UR_Message_Listener::wrench_callback, this);
  sub_joint_state_ = nh_.subscribe(topic_joint_state_, 100, &UR_Message_Listener::joint_state_callback, this);
  sub_joint_cmd_ = nh_.subscribe(topic_joint_cmd_, 100, &UR_Message_Listener::joint_cmd_callback, this);

  ROS_INFO_STREAM("The recorder node is created at: " << nh_.getNamespace() << " with freq: " << frequency
                                                      << " Hz prefix: " << ur_namespace_);
  generate_logfile();

  // Measured joint variables
  // state_sub_ = nh_.subscribe("/joint_states", 1, &UR_Message_Listener::joint_state_Callback, this);
  // It takes a long time to get message from this topic, about 0.5s

  pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(ur_namespace_ + "/pose", 10,
                                                        &UR_Message_Listener::m_tcp_pose_Callback, this);

  //   sub_wrench_external_ = nh_.subscribe<geometry_msgs::WrenchStamped>(ur_namespace_ + "/robotiq_ft_wrench", 1,
  //                                                                      &UR_Message_Listener::FT_wrench_Callback,
  //                                                                      this);
  // Measured tcp velocity
  m_tcp_speed_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(ur_namespace + "/m_tool_velocity", 10,
                                                                &UR_Message_Listener::m_tcp_speedCallback, this);

  // Joint speed command
  joint_speed_com_sub_ = nh_.subscribe<trajectory_msgs::JointTrajectory>(
      ur_namespace + "/ur_driver/joint_speed", 10, &UR_Message_Listener::c_joint_vel_Callback, this);

  c_tcp_speed_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(ur_namespace + "/c_tool_velocity", 10,
                                                                &UR_Message_Listener::c_tcp_speedCallback, this);

  execTrajectorySub_ = nh_.subscribe<moveit_msgs::RobotTrajectory>("/execute_my_trajectory", 10,
                                                                   &UR_Message_Listener::trajectoryCallback, this);

  offset_sub_ = nh_.subscribe("/offset_point", 1, &UR_Message_Listener::offset_Callback, this);

  newTrajectory = false;
  wrench_external_.setZero();
  wrench_filter_factor_ = 0.1;
  force_dead_zone_thres_ = 5;
  torque_dead_zone_thres_ = 0.5;
}

void UR_Message_Listener::c_joint_vel_Callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
  last_c_joint_vel_msg_ = *msg;
}

// void UR_Message_Listener::joint_state_Callback(const sensor_msgs::JointState::Ptr &msg)
// {
//     std::string name = (*msg).name[0];
//     // "joint_states" receives messages of all robots. This is seperating the searched one from the others.
//     if (name.compare(0, ur_prefix_.size(), ur_prefix_) == 0)
//     {
//         last_joint_state_msg_ = *msg;
//     }
// }

// Be careful that it can't compile with const trajectory_msgs::JointTrajectory::Ptr
void UR_Message_Listener::FT_wrench_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  // last_wrench_msg_ = *msg;
  Vector6d wrench_ft_frame;
  // Reading the FT-sensor in its own frame (robotiq_ft_frame_id)
  wrench_ft_frame << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x,
      msg->wrench.torque.y, msg->wrench.torque.z;

  // Dead zone for the FT sensor
  for (int i = 0; i < 3; i++)
  {
    if (abs(wrench_ft_frame(i)) < force_dead_zone_thres_)
    {
      wrench_ft_frame(i) = 0;
    }
    if (abs(wrench_ft_frame(i + 3)) < torque_dead_zone_thres_)
    {
      wrench_ft_frame(i + 3) = 0;
    }
  }
  // ROS_INFO_STREAM("After dead zone wrench_ft_frame: \n" << wrench_ft_frame);
  // Filter and update
  wrench_external_ = (1 - wrench_filter_factor_) * wrench_external_ + wrench_filter_factor_ * wrench_ft_frame;
  last_wrench_msg_.header = msg->header;
  last_wrench_msg_.wrench.force.x = wrench_external_(0);
  last_wrench_msg_.wrench.force.y = wrench_external_(1);
  last_wrench_msg_.wrench.force.z = wrench_external_(2);
  last_wrench_msg_.wrench.torque.x = wrench_external_(3);
  last_wrench_msg_.wrench.torque.y = wrench_external_(4);
  last_wrench_msg_.wrench.torque.z = wrench_external_(5);
}
void UR_Message_Listener::m_tcp_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  last_m_tcp_pose_msg_ = *msg;
}

void UR_Message_Listener::trajectoryCallback(const moveit_msgs::RobotTrajectory::ConstPtr &msg)
{
  std::string left = "left_";
  std::string right = "right_";
  if (left.compare(0, ur_prefix_.size(), ur_prefix_) == 0 && (msg->joint_trajectory.points.size() > 0))
  {
    // "joint_states" receives messages of all robots. This is seperating the searched one from the others.
    last_trajectory_msg_ = *(msg.get());
    newTrajectory = true;
    // ROS_INFO("Header time  %f ", last_trajectory_msg_.joint_trajectory.header.stamp.toSec());
    // for (unsigned int i = 0; i < last_trajectory_msg_.joint_trajectory.points.size(); i++){
    //     ROS_INFO("Listening Points %d  %f ", i,
    //     last_trajectory_msg_.joint_trajectory.header.stamp.toSec()+last_trajectory_msg_.joint_trajectory.points[i].time_from_start.toSec());
    //     for (unsigned int a = 0; a < last_trajectory_msg_.joint_trajectory.points[i].positions.size(); a++){
    //         ROS_INFO("%s:\tpos %f\tvel %f",
    //         last_trajectory_msg_.joint_trajectory.joint_names[a].c_str(),
    //         last_trajectory_msg_.joint_trajectory.points[i].positions[a],
    //         last_trajectory_msg_.joint_trajectory.points[i].velocities[a]);
    //     }
    // }
  }
  else if (right.compare(0, ur_prefix_.size(), ur_prefix_) == 0 && (msg->joint_trajectory.points.size() > 0))
  {
    last_trajectory_msg_ = *(msg.get());
    newTrajectory = true;
  }
}

void UR_Message_Listener::m_tcp_speedCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  ROS_INFO("I heared m_tcp_speedCallback!");
  last_m_tcp_vel_msg_ = *msg;
}
void UR_Message_Listener::c_tcp_speedCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  ROS_INFO("I heared c_tcp_speedCallback!");
  last_c_tcp_vel_msg_ = *msg;
}

void UR_Message_Listener::offset_Callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  last_offset_msg_ = *msg;
}

///////////////////////////////////////////////////////////////
////////////////////////// Callbacks //////////////////////////
///////////////////////////////////////////////////////////////

void UR_Message_Listener::cartesian_state_callback(const cartesian_state_msgs::PoseTwistConstPtr msg)
{
  file_cartesian_state_ << ros::Time::now() << delimiter_ << msg->pose.position.x << delimiter_ << msg->pose.position.y
                        << delimiter_ << msg->pose.position.z << delimiter_ << msg->pose.orientation.x << delimiter_
                        << msg->pose.orientation.y << delimiter_ << msg->pose.orientation.z << delimiter_
                        << msg->pose.orientation.w << delimiter_ << msg->twist.linear.x << delimiter_
                        << msg->twist.linear.y << delimiter_ << msg->twist.linear.z << delimiter_
                        << msg->twist.angular.x << delimiter_ << msg->twist.angular.y << delimiter_
                        << msg->twist.angular.z << "\n";
}
void UR_Message_Listener::cartesian_cmd_callback(const geometry_msgs::TwistConstPtr &msg)
{
  file_cartesian_cmd_ << ros::Time::now() << delimiter_ << msg->linear.x << delimiter_ << msg->linear.y << delimiter_
                      << msg->linear.z << delimiter_ << msg->angular.x << delimiter_ << msg->angular.y << delimiter_
                      << msg->angular.z << "\n";
}
void UR_Message_Listener::wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg)
{
  last_wrench_msg_ = *msg;
  file_wrench_ << ros::Time::now() << delimiter_ << msg->wrench.force.x << delimiter_ << msg->wrench.force.y
               << delimiter_ << msg->wrench.force.z << delimiter_ << msg->wrench.torque.x << delimiter_
               << msg->wrench.torque.y << delimiter_ << msg->wrench.torque.z << "\n";
}
void UR_Message_Listener::joint_state_callback(const sensor_msgs::JointState::Ptr &msg)
{
  std::string name = (*msg).name[0];
  // "joint_states" receives messages of all robots. This is seperating the searched one from the others.
  if (name.compare(0, ur_namespace_.size(), ur_namespace_) == 0)
  {
    last_joint_state_msg_ = *msg;
    file_joint_state_ << ros::Time::now();
    if (msg->position.size() == 6)
    {
      for (int i = 0; i < 6; i++)
      {
        file_joint_state_ << delimiter_ << msg->position[i];
      }
    }
    else
    {
      file_joint_state_ << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_;
    }
    // append joint velocity
    if (msg->velocity.size() == 6)
    {
      for (int i = 0; i < 6; i++)
      {
        file_joint_state_ << delimiter_ << msg->velocity[i];
      }
    }
    else
    {
      file_joint_state_ << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_;
    }
  }
}
void UR_Message_Listener::joint_cmd_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
  last_joint_cmd_msg_ = *msg;
}

void UR_Message_Listener::generate_logfile()
{  // automatically generate a name

  // Creating a recording directory
  //   std::string recPath_ = "/home/chunting/catkin_ws/src/dual_arm_manipulation/dataLog/";
  //   time_t rawtime;
  //   struct tm *timeinfo;
  //   char buffer[20];

  //   time(&rawtime);
  //   timeinfo = localtime(&rawtime);

  //   strftime(buffer, 80, "%Y_%m_%d_%H_%M", timeinfo);
  //   recPath_ = recPath_ + std::string(buffer) + '/';
  //   const int dir_err = mkdir(recPath_.c_str(), 0777);
  std::string logfile_path = folder_name_ + ur_namespace_;
  std::string logfile_name_ = logfile_path + "_cartesian_state.csv";
  file_cartesian_state_.open(logfile_name_.c_str(), std::ofstream::out | std::ofstream::trunc);
  if (!file_cartesian_state_.is_open())
  {
    ROS_ERROR("Failed to open %s", logfile_name_.c_str());
  }

  file_cartesian_state_ << "Time" << delimiter_ << ur_namespace_ + "_state_x" << delimiter_
                        << ur_namespace_ + "_state_y" << delimiter_ << ur_namespace_ + "_state_z" << delimiter_
                        << ur_namespace_ + "_state_w" << delimiter_ << ur_namespace_ + "_state_rx" << delimiter_
                        << ur_namespace_ + "_state_ry" << delimiter_ << ur_namespace_ + "_state_rz" << delimiter_
                        << ur_namespace_ + "_state_vx" << delimiter_ << ur_namespace_ + "_state_vy" << delimiter_
                        << ur_namespace_ + "_state_vz" << delimiter_ << ur_namespace_ + "_state_wx" << delimiter_
                        << ur_namespace_ + "_state_wy" << delimiter_ << ur_namespace_ + "_state_wz"
                        << "\n";

  logfile_name_ = logfile_path + "_cartesian_cmd.csv";
  file_cartesian_cmd_.open(logfile_name_.c_str(), std::ofstream::out | std::ofstream::trunc);
  if (!file_cartesian_cmd_.is_open())
  {
    ROS_ERROR("Failed to open %s", logfile_name_.c_str());
  }
  file_cartesian_cmd_ << "Time" << delimiter_ << ur_namespace_ + "_cmd_vx" << delimiter_ << ur_namespace_ + "_cmd_vy"
                      << delimiter_ << ur_namespace_ + "_cmd_vz" << delimiter_ << ur_namespace_ + "_cmd_wx"
                      << delimiter_ << ur_namespace_ + "_cmd_wy" << delimiter_ << ur_namespace_ + "_cmd_wz"
                      << "\n";

  logfile_name_ = logfile_path + "_wrench.csv";
  file_wrench_.open(logfile_name_.c_str(), std::ofstream::out | std::ofstream::trunc);
  if (!file_wrench_.is_open())
  {
    ROS_ERROR("Failed to open %s", logfile_name_.c_str());
  }
  file_wrench_ << "Time" << delimiter_ << ur_namespace_ + "_Fx" << delimiter_ << ur_namespace_ + "_Fy" << delimiter_
               << ur_namespace_ + "_Fz" << delimiter_ << ur_namespace_ + "_Tx" << delimiter_ << ur_namespace_ + "_Ty"
               << delimiter_ << ur_namespace_ + "_Tz"
               << "\n";
  logfile_name_ = logfile_path + "_joint_state.csv";
  file_joint_state_.open(logfile_name_.c_str(), std::ofstream::out | std::ofstream::trunc);
  if (!file_joint_state_.is_open())
  {
    ROS_ERROR("Failed to open %s", logfile_name_.c_str());
  }
  file_joint_state_ << "Time" << delimiter_ << ur_namespace_ + "_joint_pos_1" << delimiter_
                    << ur_namespace_ + "_joint_pos_2" << delimiter_ << ur_namespace_ + "_joint_pos_3" << delimiter_
                    << ur_namespace_ + "_joint_pos_4" << delimiter_ << ur_namespace_ + "_joint_pos_5" << delimiter_
                    << ur_namespace_ + "_joint_pos_6" << delimiter_ << ur_namespace_ + "_joint_vel_1" << delimiter_
                    << ur_namespace_ + "_joint_vel_2" << delimiter_ << ur_namespace_ + "_joint_vel_3" << delimiter_
                    << ur_namespace_ + "_joint_vel_4" << delimiter_ << ur_namespace_ + "_joint_vel_5" << delimiter_
                    << ur_namespace_ + "_joint_vel_6"
                    << "\n";
  logfile_name_ = logfile_path + "_joint_cmd.csv";
  file_joint_cmd_.open(logfile_name_.c_str(), std::ofstream::out | std::ofstream::trunc);
  if (!file_joint_cmd_.is_open())
  {
    ROS_ERROR("Failed to open %s", logfile_name_.c_str());
  }
  file_joint_cmd_ << "Time" << delimiter_ << ur_namespace_ + "_joint_cmd_1" << delimiter_
                  << ur_namespace_ + "_joint_cmd_2" << delimiter_ << ur_namespace_ + "_joint_cmd_3" << delimiter_
                  << ur_namespace_ + "_joint_cmd_4" << delimiter_ << ur_namespace_ + "_joint_cmd_5" << delimiter_
                  << ur_namespace_ + "_joint_cmd_6"
                  << "\n";
}
