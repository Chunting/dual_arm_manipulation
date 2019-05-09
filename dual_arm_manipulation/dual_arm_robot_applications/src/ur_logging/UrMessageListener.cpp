#include "ur_logging/UrMessageListener.h"

UR_Message_Listener::UR_Message_Listener(ros::NodeHandle &nh, std::string ur_namespace, std::string folder_name)
    : nh_(nh), ur_namespace_(ur_namespace), folder_name_(folder_name)
{
    stopwatch_.restart();
    if (ur_namespace_.size() > 0)
    {
        ur_prefix_ = ur_namespace_ + "_";
    }
    topic_cartesian_state_ = ur_namespace_ + "/ur5_cartesian_velocity_controller/ee_state";
    topic_cartesian_pos_state_ = ur_namespace_ + "/tool_pose";
    topic_cartesian_vel_state_ = ur_namespace_ + "/tool_velocity";
    topic_cartesian_vel_cmd_ = ur_namespace_ + "/ur5_cartesian_velocity_controller/command_cart_vel";
    topic_cartesian_pos_cmd_ = ur_namespace_ + "/command_cart_pos";
    topic_external_wrench = ur_namespace_ + "/robotiq_ft_wrench";
    topic_joint_state_ = "/joint_states";
    topic_joint_traj_cmd_ = ur_namespace + "/joint_traj_cmd";

    delimiter_ = ',';

    sub_cartesian_state_ = nh_.subscribe(topic_cartesian_state_, 100, &UR_Message_Listener::cartesian_state_callback, this);
    sub_cartesian_vel_state_ = nh_.subscribe(topic_cartesian_vel_state_, 100, &UR_Message_Listener::cartesian_vel_state_callback, this);
    sub_cartesian_pos_state_ = nh_.subscribe(topic_cartesian_pos_state_, 100, &UR_Message_Listener::cartesian_pos_state_callback, this);
    sub_cartesian_vel_cmd_ = nh_.subscribe(topic_cartesian_vel_cmd_, 100, &UR_Message_Listener::cartesian_vel_cmd_callback, this);
    sub_cartesian_pos_cmd_ = nh_.subscribe(topic_cartesian_pos_cmd_, 100, &UR_Message_Listener::cartesian_pos_cmd_callback, this);
    sub_wrench_external_ = nh_.subscribe(topic_external_wrench, 100, &UR_Message_Listener::wrench_callback, this);
    sub_joint_state_ = nh_.subscribe(topic_joint_state_, 100, &UR_Message_Listener::joint_state_callback, this);
    sub_joint_traj_cmd_ = nh_.subscribe<trajectory_msgs::JointTrajectory>(topic_joint_traj_cmd_, 100, &UR_Message_Listener::joint_traj_cmd_callback, this);

    generate_logfile();
    // Joint speed command
    joint_speed_com_sub_ = nh_.subscribe<trajectory_msgs::JointTrajectory>(
        ur_namespace + "/ur_driver/joint_speed", 100, &UR_Message_Listener::c_joint_vel_Callback, this);

    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(ur_namespace_ + "/ee_pose_in_world", 100,
                                                          &UR_Message_Listener::m_tcp_pose_Callback, this);
    // Measured tcp velocity
    m_tcp_speed_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(ur_namespace + "/tool_velocity", 100,
                                                                  &UR_Message_Listener::m_tcp_speedCallback, this);
    offset_sub_ = nh_.subscribe("/real_time_offset_point", 1, &UR_Message_Listener::offset_Callback, this);

    c_tcp_speed_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(ur_namespace + "/c_tool_velocity", 100,
                                                                  &UR_Message_Listener::c_tcp_speedCallback, this);

    execTrajectorySub_ = nh_.subscribe<moveit_msgs::RobotTrajectory>("/execute_my_trajectory", 100,
                                                                     &UR_Message_Listener::trajectoryCallback, this);

    newTrajectory = false;
    wrench_external_.setZero();
    wrench_filter_factor_ = 0.1;
    force_dead_zone_thres_ = 5;
    torque_dead_zone_thres_ = 0.1;
   
    start_secondes_ = (ros::Time::now()).toSec();
}

void UR_Message_Listener::c_joint_vel_Callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
    last_c_joint_vel_msg_ = *msg;
}

// Be careful that it can't compile with const trajectory_msgs::JointTrajectory::Ptr
void UR_Message_Listener::FT_wrench_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    Vector6d wrench_ft_frame;
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
    last_m_tcp_vel_msg_ = *msg;
}
void UR_Message_Listener::c_tcp_speedCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    last_c_tcp_vel_msg_ = *msg;
}

void UR_Message_Listener::offset_Callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    last_offset_msg_ = *msg;
}

///////////////////////////////////////////////////////////////
////////////////////////// Callbacks //////////////////////////
///////////////////////////////////////////////////////////////

void UR_Message_Listener::cartesian_state_callback(const cartesian_state_msgs::PoseTwistConstPtr &msg)
{
    last_cartesian_state_msg_ = *msg;
}
void UR_Message_Listener::cartesian_vel_state_callback(const geometry_msgs::TwistStampedConstPtr &msg)
{
    last_cartesian_vel_state_msg_ = *msg;
}
void UR_Message_Listener::cartesian_pos_state_callback(const geometry_msgs::PoseStampedPtr &msg)
{
    last_cartesian_pos_state_msg_ = *msg;
}
void UR_Message_Listener::cartesian_vel_cmd_callback(const geometry_msgs::TwistConstPtr &msg)
{
    last_cartesian_vel_cmd_msg_ = *msg;
}
void UR_Message_Listener::cartesian_pos_cmd_callback(const geometry_msgs::PoseStampedPtr &msg)
{
    last_cartesian_pos_cmd_msg_ = *msg;
}
void UR_Message_Listener::wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg)
{
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
void UR_Message_Listener::joint_state_callback(const sensor_msgs::JointState::Ptr &msg)
{
    // "joint_states" receives messages of all robots. This is seperating the searched one from the others.
    if (msg->position.size() > 0 && msg->name[0].find(ur_namespace_) != std::string::npos)
    {
        last_joint_state_msg_ = *msg;
    }
}
void UR_Message_Listener::joint_traj_cmd_callback(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
    last_joint_traj_cmd_msg_ = *msg;
}

void UR_Message_Listener::generate_logfile()
{ // automatically generate a name

    std::string logfile_path = folder_name_ + ur_namespace_;
    std::string logfile_name = logfile_path + "_cartesian_state.csv";
    file_cartesian_state_.open(logfile_name.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!file_cartesian_state_.is_open())
    {
        ROS_ERROR("Failed to open %s", logfile_name.c_str());
    }

    file_cartesian_state_ << "Time" << delimiter_ << ur_namespace_ + "_state_x" << delimiter_
                          << ur_namespace_ + "_state_y" << delimiter_ << ur_namespace_ + "_state_z" << delimiter_
                          << ur_namespace_ + "_state_w" << delimiter_ << ur_namespace_ + "_state_rx" << delimiter_
                          << ur_namespace_ + "_state_ry" << delimiter_ << ur_namespace_ + "_state_rz" << delimiter_
                          << ur_namespace_ + "_state_vx" << delimiter_ << ur_namespace_ + "_state_vy" << delimiter_
                          << ur_namespace_ + "_state_vz" << delimiter_ << ur_namespace_ + "_state_wx" << delimiter_
                          << ur_namespace_ + "_state_wy" << delimiter_ << ur_namespace_ + "_state_wz"
                          << "\n";
    // Create log file for Cartesian information
    logfile_name = logfile_path + "_cartesian_vel_state.csv";
    file_cartesian_vel_state_.open(logfile_name.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!file_cartesian_vel_state_.is_open())
    {
        ROS_ERROR("Failed to open %s", logfile_name.c_str());
    }
    file_cartesian_vel_state_ << "Time" << delimiter_ << ur_namespace_ + "_state_vx" << delimiter_ << ur_namespace_ + "_state_vy"
                              << delimiter_ << ur_namespace_ + "_state_vz" << delimiter_ << ur_namespace_ + "_state_wx"
                              << delimiter_ << ur_namespace_ + "_state_wy" << delimiter_ << ur_namespace_ + "_state_wz"
                              << "\n";

    logfile_name = logfile_path + "_cartesian_pos_state.csv";
    file_cartesian_pos_state_.open(logfile_name.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!file_cartesian_pos_cmd_.is_open())
    {
        ROS_ERROR("Failed to open %s", logfile_name.c_str());
    }
    file_cartesian_pos_state_ << "Time" << delimiter_ << ur_namespace_ + "_state_x" << delimiter_ << ur_namespace_ + "_state_y"
                              << delimiter_ << ur_namespace_ + "_state_z" << delimiter_ << ur_namespace_ + "_state_qx"
                              << delimiter_ << ur_namespace_ + "_state_qy" << delimiter_ << ur_namespace_ + "_state_qz"
                              << delimiter_ << ur_namespace_ + "_state_qw"
                              << "\n";
    // Create log file for cartesian command
    logfile_name = logfile_path + "_cartesian_vel_cmd.csv";
    file_cartesian_vel_cmd_.open(logfile_name.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!file_cartesian_vel_cmd_.is_open())
    {
        ROS_ERROR("Failed to open %s", logfile_name.c_str());
    }
    file_cartesian_vel_cmd_ << "Time" << delimiter_ << ur_namespace_ + "_cmd_vx" << delimiter_ << ur_namespace_ + "_cmd_vy"
                            << delimiter_ << ur_namespace_ + "_cmd_vz" << delimiter_ << ur_namespace_ + "_cmd_wx"
                            << delimiter_ << ur_namespace_ + "_cmd_wy" << delimiter_ << ur_namespace_ + "_cmd_wz"
                            << "\n";

    logfile_name = logfile_path + "_cartesian_pos_cmd.csv";
    file_cartesian_pos_cmd_.open(logfile_name.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!file_cartesian_pos_cmd_.is_open())
    {
        ROS_ERROR("Failed to open %s", logfile_name.c_str());
    }
    file_cartesian_pos_cmd_ << "Time" << delimiter_ << ur_namespace_ + "_cmd_x" << delimiter_ << ur_namespace_ + "_cmd_y"
                            << delimiter_ << ur_namespace_ + "_cmd_z" << delimiter_ << ur_namespace_ + "_cmd_qx"
                            << delimiter_ << ur_namespace_ + "_cmd_qy" << delimiter_ << ur_namespace_ + "_cmd_qz"
                            << delimiter_ << ur_namespace_ + "_cmd_qw"
                            << "\n";

    // Create log file for end-effector wrench data in robotiq_ft_frame_id frame
    logfile_name = logfile_path + "_wrench.csv";
    file_wrench_.open(logfile_name.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!file_wrench_.is_open())
    {
        ROS_ERROR("Failed to open %s", logfile_name.c_str());
    }
    file_wrench_ << "Time" << delimiter_ << ur_namespace_ + "_Fx" << delimiter_ << ur_namespace_ + "_Fy" << delimiter_
                 << ur_namespace_ + "_Fz" << delimiter_ << ur_namespace_ + "_Tx" << delimiter_ << ur_namespace_ + "_Ty"
                 << delimiter_ << ur_namespace_ + "_Tz"
                 << "\n";

    // Create *_joint_state.csv
    std::vector<std::string> joint_names;
    nh_.getParam(ur_namespace_ + "/hardware_interface/joints", joint_names);
    if (joint_names.size() < 6)
    {
        ROS_ERROR("UR Logger: could not properly load joint names");
    }
    logfile_name = logfile_path + "_joint_state.csv";
    file_joint_state_.open(logfile_name.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!file_joint_state_.is_open())
    {
        ROS_ERROR("Failed to open %s", logfile_name.c_str());
    }
    file_joint_state_ << "Time";
    for (auto &jointname : joint_names)
    {
        file_joint_state_ << delimiter_ << jointname + "_state_pos";
    }
    for (auto &jointname : joint_names)
    {
        file_joint_state_ << delimiter_ << jointname + "_state_vel";
    }
    file_joint_state_ << "\n";

    // Create *_joint_cmd.csv
    logfile_name = logfile_path + "_joint_cmd.csv";
    file_joint_cmd_.open(logfile_name.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!file_joint_cmd_.is_open())
    {
        ROS_ERROR("Failed to open %s", logfile_name.c_str());
    }
    file_joint_cmd_ << "Time" << delimiter_ << "time_from_start";
    for (auto &jointname : joint_names)
    {
        file_joint_cmd_ << delimiter_ << jointname + "_cmd_pos";
    }
    for (auto &jointname : joint_names)
    {
        file_joint_cmd_ << delimiter_ << jointname + "_cmd_vel";
    }
    file_joint_cmd_ << "\n";
}
void UR_Message_Listener::write_logfile()
{
    file_cartesian_state_ << stopwatch_.elapsed().toSec()
                          << delimiter_ << last_cartesian_state_msg_.pose.position.x
                          << delimiter_ << last_cartesian_state_msg_.pose.position.y
                          << delimiter_ << last_cartesian_state_msg_.pose.position.z
                          << delimiter_ << last_cartesian_state_msg_.pose.orientation.x
                          << delimiter_ << last_cartesian_state_msg_.pose.orientation.y
                          << delimiter_ << last_cartesian_state_msg_.pose.orientation.z
                          << delimiter_ << last_cartesian_state_msg_.pose.orientation.w
                          << delimiter_ << last_cartesian_state_msg_.twist.linear.x
                          << delimiter_ << last_cartesian_state_msg_.twist.linear.y
                          << delimiter_ << last_cartesian_state_msg_.twist.linear.z
                          << delimiter_ << last_cartesian_state_msg_.twist.angular.x
                          << delimiter_ << last_cartesian_state_msg_.twist.angular.y
                          << delimiter_ << last_cartesian_state_msg_.twist.angular.z << "\n";
    file_cartesian_vel_state_ << stopwatch_.elapsed().toSec()
                              << delimiter_ << last_cartesian_vel_state_msg_.twist.linear.x
                              << delimiter_ << last_cartesian_vel_state_msg_.twist.linear.y
                              << delimiter_ << last_cartesian_vel_state_msg_.twist.linear.z
                              << delimiter_ << last_cartesian_vel_state_msg_.twist.angular.x
                              << delimiter_ << last_cartesian_vel_state_msg_.twist.angular.y
                              << delimiter_ << last_cartesian_vel_state_msg_.twist.angular.z << "\n";

    file_cartesian_pos_state_ << stopwatch_.elapsed().toSec()
                              << delimiter_ << last_cartesian_pos_state_msg_.pose.position.x
                              << delimiter_ << last_cartesian_pos_state_msg_.pose.position.y
                              << delimiter_ << last_cartesian_pos_state_msg_.pose.position.z
                              << delimiter_ << last_cartesian_pos_state_msg_.pose.orientation.x
                              << delimiter_ << last_cartesian_pos_state_msg_.pose.orientation.y
                              << delimiter_ << last_cartesian_pos_state_msg_.pose.orientation.z
                              << delimiter_ << last_cartesian_pos_state_msg_.pose.orientation.w << "\n";
    file_cartesian_vel_cmd_ << stopwatch_.elapsed().toSec()
                            << delimiter_ << last_cartesian_vel_cmd_msg_.linear.x
                            << delimiter_ << last_cartesian_vel_cmd_msg_.linear.y
                            << delimiter_ << last_cartesian_vel_cmd_msg_.linear.z
                            << delimiter_ << last_cartesian_vel_cmd_msg_.angular.x
                            << delimiter_ << last_cartesian_vel_cmd_msg_.angular.y
                            << delimiter_ << last_cartesian_vel_cmd_msg_.angular.z << "\n";
    file_cartesian_pos_cmd_ << stopwatch_.elapsed().toSec()
                            << delimiter_ << last_cartesian_pos_cmd_msg_.pose.position.x
                            << delimiter_ << last_cartesian_pos_cmd_msg_.pose.position.y
                            << delimiter_ << last_cartesian_pos_cmd_msg_.pose.position.z
                            << delimiter_ << last_cartesian_pos_cmd_msg_.pose.orientation.x
                            << delimiter_ << last_cartesian_pos_cmd_msg_.pose.orientation.y
                            << delimiter_ << last_cartesian_pos_cmd_msg_.pose.orientation.z
                            << delimiter_ << last_cartesian_pos_cmd_msg_.pose.orientation.w << "\n";
    file_wrench_ << stopwatch_.elapsed().toSec();
    for (int i = 0; i < 6; ++i)
    {
        file_wrench_ << delimiter_ << wrench_external_(i);
    }
    file_wrench_ << "\n";

    file_joint_state_ << stopwatch_.elapsed().toSec();
    for(int i=0; i<last_joint_state_msg_.position.size(); ++i)
    {
        file_joint_state_ << delimiter_ << last_joint_state_msg_.position[i];
    }

    // append joint velocity

     for(int i=0; i<last_joint_state_msg_.velocity.size(); ++i)
    {
        file_joint_state_ << delimiter_ << last_joint_state_msg_.velocity[i];
    }
    file_joint_state_ << "\n";

    for (auto &point : last_joint_traj_cmd_msg_.points)
    {
        file_joint_cmd_ << stopwatch_.elapsed().toSec() << delimiter_ << point.time_from_start;
        for (auto &pos : point.positions)
        {
            file_joint_cmd_ << delimiter_ << pos;
        }
        for (auto &vel : point.velocities)
        {
            file_joint_cmd_ << delimiter_ << vel;
        }
        file_joint_cmd_ << "\n";
    }
}
