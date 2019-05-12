#include "ur_logging/UrMessageListener.h"

UR_Message_Listener::UR_Message_Listener(ros::NodeHandle &nh, std::string ur_namespace, std::string folder_name)
    : nh_(nh), ur_namespace_(ur_namespace), folder_name_(folder_name)
{
    stopwatch_.restart();

    topic_cart_state_ = ur_namespace_ + "/ur5_cartesian_velocity_controller/ee_state";
    topic_cart_vel_cmd_ = ur_namespace_ + "/ur5_cartesian_velocity_controller/command_cart_vel";

    topic_cart_pose_state_ = ur_namespace_ + "/tool_pose";
    topic_cart_vel_state_ = ur_namespace_ + "/tool_velocity";
    topic_cart_pose_cmd_ = ur_namespace_ + "/command_cart_pos";
    topic_external_wrench = ur_namespace_ + "/robotiq_ft_wrench";
    topic_joint_traj_cmd_ = ur_namespace_ + "/joint_traj_cmd";
    topic_joint_state_ = ur_namespace_ + "/joint_states";
    topic_robot_traj_cmd_ = "/robot_traj_cmd";
    topic_offset_point_state_ = "/offset_point_state";


    sub_cart_state_         = nh_.subscribe<cartesian_state_msgs::PoseTwist>(topic_cart_state_, 100, &UR_Message_Listener::tool_state_callback, this);
    sub_cart_vel_cmd_       = nh_.subscribe<geometry_msgs::Twist>(topic_cart_vel_cmd_, 100, &UR_Message_Listener::cart_vel_cmd_callback, this);
    sub_cart_vel_state_     = nh_.subscribe<geometry_msgs::TwistStamped>(topic_cart_vel_state_, 100, &UR_Message_Listener::cart_vel_state_callback, this);
    sub_cart_pose_state_    = nh_.subscribe<geometry_msgs::PoseStamped>(topic_cart_pose_state_, 100, &UR_Message_Listener::cart_pose_state_callback, this);
    sub_cart_pose_cmd_      = nh_.subscribe<geometry_msgs::PoseStamped>(topic_cart_pose_cmd_, 100, &UR_Message_Listener::cartesian_pose_cmd_callback, this);
    sub_wrench_external_    = nh_.subscribe<geometry_msgs::WrenchStamped>(topic_external_wrench, 100, &UR_Message_Listener::wrench_callback, this);
    sub_joint_state_        = nh_.subscribe<sensor_msgs::JointState>(topic_joint_state_, 100, &UR_Message_Listener::joint_state_callback, this);
    sub_joint_traj_cmd_     = nh_.subscribe<trajectory_msgs::JointTrajectory>(topic_joint_traj_cmd_, 1, &UR_Message_Listener::joint_traj_cmd_callback, this);
    sub_offset_point_state_ = nh_.subscribe<geometry_msgs::PointStamped>(topic_offset_point_state_, 1, &UR_Message_Listener::offset_point_state_callback, this);
    sub_robot_traj_cmd_     = nh_.subscribe<moveit_msgs::RobotTrajectory>(topic_robot_traj_cmd_, 100, &UR_Message_Listener::robot_traj_cmd_callback, this);
    pub_joint_state_        = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    start_time_ = ros::Time::now().toSec();
    delimiter_ = ',';
    wrench_external_.setZero();
    wrench_filter_factor_ = 0.1;
    force_dead_zone_thres_ = 5;
    torque_dead_zone_thres_ = 0.1;
    newTrajectory = false;
}
bool UR_Message_Listener::waitForValid(double seconds)
{
    ros::Duration(seconds).sleep();
    if(last_joint_state_msg_.header.stamp.waitForValid()){
        start_time_ = last_joint_state_msg_.header.stamp.toSec();
    } else {
        ROS_ERROR("There is no joint state message coming...");
        return false;
    }
    if(last_joint_state_msg_.header.stamp.waitForValid()){
        start_time_ = last_joint_state_msg_.header.stamp.toSec();
    } else {
        ROS_ERROR("There is no joint state message coming...");
        return false;
    }
    if(last_cart_vel_state_msg_.header.stamp.waitForValid()){
        start_time_ = std::min(last_cart_vel_state_msg_.header.stamp.toSec(), start_time_);
    } else {
        ROS_ERROR("There is no Cartesian message coming...");
        return false;
    }
    pre_cmd_time_ = start_time_;
    ROS_ERROR("start_time_ = %f", start_time_ );
    return true;
}
void UR_Message_Listener::start()
{
    if(waitForValid()){
        generate_logfile();
    } else{
        ROS_ERROR("Please wait for message to come...");
    }
}


void UR_Message_Listener::robot_traj_cmd_callback(const moveit_msgs::RobotTrajectory::ConstPtr &msg)
{
    std::string left = "left";
    std::string right = "right";
    if (left.compare(0, ur_namespace_.size(), ur_namespace_) == 0 && (msg->joint_trajectory.points.size() > 0))
    {
        // "joint_states" receives messages of all robots. This is seperating the searched one from the others.
        last_robot_traj_cmd_msg_ = *(msg.get());
        newTrajectory = true;
        // ROS_INFO("Header time  %f ", last_robot_traj_cmd_msg_.joint_trajectory.header.stamp.toSec());
        // for (unsigned int i = 0; i < last_robot_traj_cmd_msg_.joint_trajectory.points.size(); i++){
        //     ROS_INFO("Listening Points %d  %f ", i,
        //     last_robot_traj_cmd_msg_.joint_trajectory.header.stamp.toSec()+last_robot_traj_cmd_msg_.joint_trajectory.points[i].time_from_start.toSec());
        //     for (unsigned int a = 0; a < last_robot_traj_cmd_msg_.joint_trajectory.points[i].positions.size(); a++){
        //         ROS_INFO("%s:\tpos %f\tvel %f",
        //         last_robot_traj_cmd_msg_.joint_trajectory.joint_names[a].c_str(),
        //         last_robot_traj_cmd_msg_.joint_trajectory.points[i].positions[a],
        //         last_robot_traj_cmd_msg_.joint_trajectory.points[i].velocities[a]);
        //     }
        // }
    }
    else if (right.compare(0, ur_namespace_.size(), ur_namespace_) == 0 && (msg->joint_trajectory.points.size() > 0))
    {
        last_robot_traj_cmd_msg_ = *(msg.get());
        newTrajectory = true;
    }
}

void UR_Message_Listener::offset_point_state_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    last_offset_point_state_msg_ = *msg;
}

///////////////////////////////////////////////////////////////
////////////////////////// Callbacks //////////////////////////
///////////////////////////////////////////////////////////////

void UR_Message_Listener::tool_state_callback(const cartesian_state_msgs::PoseTwist::ConstPtr &msg)
{
    last_cart_state_msg_ = *msg;
}
void UR_Message_Listener::cart_vel_state_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    last_cart_vel_state_msg_ = *msg;
}
void UR_Message_Listener::cart_pose_state_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    last_cart_pose_state_msg_ = *msg;
}
void UR_Message_Listener::cart_vel_cmd_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    last_cart_vel_cmd_msg_ = *msg;
}
void UR_Message_Listener::cartesian_pose_cmd_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    last_cart_pose_cmd_msg_ = *msg;
}
void UR_Message_Listener::wrench_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
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
void UR_Message_Listener::joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    // "joint_states" receives messages of all robots. This is seperating the searched one from the others.
    if (msg->position.size() > 0 && msg->name[0].find(ur_namespace_) != std::string::npos)
    {
        last_joint_state_msg_ = *msg;
    }
    pub_joint_state_.publish(last_joint_state_msg_);
}
void UR_Message_Listener::joint_traj_cmd_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
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
    logfile_name = logfile_path + "_cart_vel_state.csv";
    file_cart_vel_state_.open(logfile_name.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!file_cart_vel_state_.is_open())
    {
        ROS_ERROR("Failed to open %s", logfile_name.c_str());
    }
    file_cart_vel_state_ << "Time" << delimiter_ << ur_namespace_ + "_state_vx" << delimiter_ << ur_namespace_ + "_state_vy"
                              << delimiter_ << ur_namespace_ + "_state_vz" << delimiter_ << ur_namespace_ + "_state_wx"
                              << delimiter_ << ur_namespace_ + "_state_wy" << delimiter_ << ur_namespace_ + "_state_wz"
                              << "\n";

    logfile_name = logfile_path + "_cart_pose_state.csv";
    file_cart_pose_state_.open(logfile_name.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!file_cart_pose_cmd_.is_open())
    {
        ROS_ERROR("Failed to open %s", logfile_name.c_str());
    }
    file_cart_pose_state_ << "Time" << delimiter_ << ur_namespace_ + "_state_x" << delimiter_ << ur_namespace_ + "_state_y"
                              << delimiter_ << ur_namespace_ + "_state_z" << delimiter_ << ur_namespace_ + "_state_qx"
                              << delimiter_ << ur_namespace_ + "_state_qy" << delimiter_ << ur_namespace_ + "_state_qz"
                              << delimiter_ << ur_namespace_ + "_state_qw"
                              << "\n";
    // Create log file for cartesian command
    logfile_name = logfile_path + "_cart_vel_cmd.csv";
    file_cart_vel_cmd_.open(logfile_name.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!file_cart_vel_cmd_.is_open())
    {
        ROS_ERROR("Failed to open %s", logfile_name.c_str());
    }
    file_cart_vel_cmd_ << "Time" << delimiter_ << ur_namespace_ + "_cmd_vx" << delimiter_ << ur_namespace_ + "_cmd_vy"
                            << delimiter_ << ur_namespace_ + "_cmd_vz" << delimiter_ << ur_namespace_ + "_cmd_wx"
                            << delimiter_ << ur_namespace_ + "_cmd_wy" << delimiter_ << ur_namespace_ + "_cmd_wz"
                            << "\n";

    logfile_name = logfile_path + "_cart_pose_cmd.csv";
    file_cart_pose_cmd_.open(logfile_name.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!file_cart_pose_cmd_.is_open())
    {
        ROS_ERROR("Failed to open %s", logfile_name.c_str());
    }
    file_cart_pose_cmd_ << "Time" << delimiter_ << ur_namespace_ + "_cmd_x" << delimiter_ << ur_namespace_ + "_cmd_y"
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
    std::copy(last_joint_state_msg_.name.begin(), last_joint_state_msg_.name.end(), std::back_inserter(joint_names));
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
    file_cartesian_state_ << last_cart_state_msg_.header.stamp.toSec() - start_time_
                          << delimiter_ << last_cart_state_msg_.pose.position.x
                          << delimiter_ << last_cart_state_msg_.pose.position.y
                          << delimiter_ << last_cart_state_msg_.pose.position.z
                          << delimiter_ << last_cart_state_msg_.pose.orientation.x
                          << delimiter_ << last_cart_state_msg_.pose.orientation.y
                          << delimiter_ << last_cart_state_msg_.pose.orientation.z
                          << delimiter_ << last_cart_state_msg_.pose.orientation.w
                          << delimiter_ << last_cart_state_msg_.twist.linear.x
                          << delimiter_ << last_cart_state_msg_.twist.linear.y
                          << delimiter_ << last_cart_state_msg_.twist.linear.z
                          << delimiter_ << last_cart_state_msg_.twist.angular.x
                          << delimiter_ << last_cart_state_msg_.twist.angular.y
                          << delimiter_ << last_cart_state_msg_.twist.angular.z << "\n";
    file_cart_vel_state_ << last_cart_vel_state_msg_.header.stamp.toSec() - start_time_
                              << delimiter_ << last_cart_vel_state_msg_.twist.linear.x
                              << delimiter_ << last_cart_vel_state_msg_.twist.linear.y
                              << delimiter_ << last_cart_vel_state_msg_.twist.linear.z
                              << delimiter_ << last_cart_vel_state_msg_.twist.angular.x
                              << delimiter_ << last_cart_vel_state_msg_.twist.angular.y
                              << delimiter_ << last_cart_vel_state_msg_.twist.angular.z << "\n";

    file_cart_pose_state_ << last_cart_pose_state_msg_.header.stamp.toSec() - start_time_
                              << delimiter_ << last_cart_pose_state_msg_.pose.position.x
                              << delimiter_ << last_cart_pose_state_msg_.pose.position.y
                              << delimiter_ << last_cart_pose_state_msg_.pose.position.z
                              << delimiter_ << last_cart_pose_state_msg_.pose.orientation.x
                              << delimiter_ << last_cart_pose_state_msg_.pose.orientation.y
                              << delimiter_ << last_cart_pose_state_msg_.pose.orientation.z
                              << delimiter_ << last_cart_pose_state_msg_.pose.orientation.w << "\n";
    file_cart_vel_cmd_ << stopwatch_.elapsed().toSec()
                            << delimiter_ << last_cart_vel_cmd_msg_.linear.x
                            << delimiter_ << last_cart_vel_cmd_msg_.linear.y
                            << delimiter_ << last_cart_vel_cmd_msg_.linear.z
                            << delimiter_ << last_cart_vel_cmd_msg_.angular.x
                            << delimiter_ << last_cart_vel_cmd_msg_.angular.y
                            << delimiter_ << last_cart_vel_cmd_msg_.angular.z << "\n";
    if(!last_cart_pose_cmd_msg_.header.stamp.is_zero()){
        file_cart_pose_cmd_ << last_cart_pose_cmd_msg_.header.stamp.toSec() - start_time_
                            << delimiter_ << last_cart_pose_cmd_msg_.pose.position.x
                            << delimiter_ << last_cart_pose_cmd_msg_.pose.position.y
                            << delimiter_ << last_cart_pose_cmd_msg_.pose.position.z
                            << delimiter_ << last_cart_pose_cmd_msg_.pose.orientation.x
                            << delimiter_ << last_cart_pose_cmd_msg_.pose.orientation.y
                            << delimiter_ << last_cart_pose_cmd_msg_.pose.orientation.z
                            << delimiter_ << last_cart_pose_cmd_msg_.pose.orientation.w << "\n";
    }

    file_wrench_ << last_cart_pose_cmd_msg_.header.stamp.toSec() - start_time_
                            << delimiter_ << last_wrench_msg_.wrench.force.x
                            << delimiter_ << last_wrench_msg_.wrench.force.y 
                            << delimiter_ << last_wrench_msg_.wrench.force.z 
                            << delimiter_ << last_wrench_msg_.wrench.torque.x 
                            << delimiter_ << last_wrench_msg_.wrench.torque.y 
                            << delimiter_ << last_wrench_msg_.wrench.torque.z << "\n";

    file_joint_state_ << last_joint_state_msg_.header.stamp.toSec() - start_time_;
    /* elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint */
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
    // file_joint_state_   << delimiter_ << last_joint_state_msg_.position[2]
    //                     << delimiter_ << last_joint_state_msg_.position[1]
    //                     << delimiter_ << last_joint_state_msg_.position[0]
    //                     << delimiter_ << last_joint_state_msg_.position[3]
    //                     << delimiter_ << last_joint_state_msg_.position[4]
    //                     << delimiter_ << last_joint_state_msg_.position[5]
    //                     << delimiter_ << last_joint_state_msg_.velocity[2]
    //                     << delimiter_ << last_joint_state_msg_.velocity[1]
    //                     << delimiter_ << last_joint_state_msg_.velocity[0]
    //                     << delimiter_ << last_joint_state_msg_.velocity[3]
    //                     << delimiter_ << last_joint_state_msg_.velocity[4]
    //                     << delimiter_ << last_joint_state_msg_.velocity[5] << "\n";

    
    if(last_joint_traj_cmd_msg_.header.stamp.toSec() > pre_cmd_time_){
        for (int i=0; i<last_joint_traj_cmd_msg_.points.size(); ++i)
        {
            auto point  = last_joint_traj_cmd_msg_.points[i];
            file_joint_cmd_ << last_joint_traj_cmd_msg_.header.stamp.toSec() - start_time_ + point.time_from_start.toSec()
                            << delimiter_ << point.time_from_start.toSec();
            for (int i=0; i<point.positions.size();++i)
            {
                file_joint_cmd_ << delimiter_ << point.positions[i];
                ROS_INFO("pos %f", point.positions[i]);
            }
            for (int i=0; i<point.velocities.size(); ++i)
            {
                file_joint_cmd_ << delimiter_ << point.velocities[i];
            }
            file_joint_cmd_ << "\n";
        }
        pre_cmd_time_ = last_joint_traj_cmd_msg_.header.stamp.toSec();
    }
}
/*
std::string UR_Message_Listener::write_cart_vel_state_line()
{
    std::ostringstream converter; // stream used to convert numbers to string
    converter << stopwatch_.elapsed().toSec()
                              << delimiter_ << last_cart_vel_state_msg_.twist.linear.x
                              << delimiter_ << last_cart_vel_state_msg_.twist.linear.y
                              << delimiter_ << last_cart_vel_state_msg_.twist.linear.z
                              << delimiter_ << last_cart_vel_state_msg_.twist.angular.x
                              << delimiter_ << last_cart_vel_state_msg_.twist.angular.y
                              << delimiter_ << last_cart_vel_state_msg_.twist.angular.z << "\n";
    return converter.str();
}
std::string UR_Message_Listener::write_cart_pose_state_line()
{
    std::ostringstream converter; // stream used to convert numbers to string
    converter << stopwatch_.elapsed().toSec()
                              << delimiter_ << last_cart_pose_state_msg_.pose.position.x
                              << delimiter_ << last_cart_pose_state_msg_.pose.position.y
                              << delimiter_ << last_cart_pose_state_msg_.pose.position.z
                              << delimiter_ << last_cart_pose_state_msg_.pose.orientation.x
                              << delimiter_ << last_cart_pose_state_msg_.pose.orientation.y
                              << delimiter_ << last_cart_pose_state_msg_.pose.orientation.z
                              << delimiter_ << last_cart_pose_state_msg_.pose.orientation.w << "\n";
    return converter.str();
}
std::string UR_Message_Listener::write_cart_vel_state_line()
{
    std::ostringstream converter; // stream used to convert numbers to string
    converter << stopwatch_.elapsed().toSec()
                              << delimiter_ << last_cart_vel_state_msg_.twist.linear.x
                              << delimiter_ << last_cart_vel_state_msg_.twist.linear.y
                              << delimiter_ << last_cart_vel_state_msg_.twist.linear.z
                              << delimiter_ << last_cart_vel_state_msg_.twist.angular.x
                              << delimiter_ << last_cart_vel_state_msg_.twist.angular.y
                              << delimiter_ << last_cart_vel_state_msg_.twist.angular.z << "\n";
    return converter.str();
}
std::string UR_Message_Listener::write_cart_vel_cmd_line()
{
    std::ostringstream converter; // stream used to convert numbers to string
    converter << stopwatch_.elapsed().toSec()
                            << delimiter_ << last_cart_vel_cmd_msg_.linear.x
                            << delimiter_ << last_cart_vel_cmd_msg_.linear.y
                            << delimiter_ << last_cart_vel_cmd_msg_.linear.z
                            << delimiter_ << last_cart_vel_cmd_msg_.angular.x
                            << delimiter_ << last_cart_vel_cmd_msg_.angular.y
                            << delimiter_ << last_cart_vel_cmd_msg_.angular.z << "\n";
    return converter.str();
}
std::string UR_Message_Listener::write_cart_pose_cmd_line()
{
    std::ostringstream converter; // stream used to convert numbers to string
    converter << stopwatch_.elapsed().toSec()
                            << delimiter_ << last_cart_pose_cmd_msg_.pose.position.x
                            << delimiter_ << last_cart_pose_cmd_msg_.pose.position.y
                            << delimiter_ << last_cart_pose_cmd_msg_.pose.position.z
                            << delimiter_ << last_cart_pose_cmd_msg_.pose.orientation.x
                            << delimiter_ << last_cart_pose_cmd_msg_.pose.orientation.y
                            << delimiter_ << last_cart_pose_cmd_msg_.pose.orientation.z
                            << delimiter_ << last_cart_pose_cmd_msg_.pose.orientation.w << "\n";
    return converter.str();
}
std::string UR_Message_Listener::write_wrench_line()
{
    std::ostringstream converter; // stream used to convert numbers to string
    converter << stopwatch_.elapsed().toSec()
                            << delimiter_ << last_wrench_msg_.wrench.force.x
                            << delimiter_ << last_wrench_msg_.wrench.force.y 
                            << delimiter_ << last_wrench_msg_.wrench.force.z 
                            << delimiter_ << last_wrench_msg_.wrench.torque.x 
                            << delimiter_ << last_wrench_msg_.wrench.torque.y 
                            << delimiter_ << last_wrench_msg_.wrench.torque.z << "\n";
    return converter.str();
}
std::string UR_Message_Listener::write_joint_state_line()
{
    std::ostringstream converter; // stream used to convert numbers to string
    converter << stopwatch_.elapsed().toSec();
    for(int i=0; i<last_joint_state_msg_.position.size(); ++i)
    {
        converter << delimiter_ << last_joint_state_msg_.position[i];
    }

    for(int i=0; i<last_joint_state_msg_.velocity.size(); ++i)
    {
        converter << delimiter_ << last_joint_state_msg_.velocity[i];
    }
    converter << "\n";
    return converter.str();
}
std::string UR_Message_Listener::write_joint_cmd_line()
{
    std::ostringstream converter; // stream used to convert numbers to string
    converter << stopwatch_.elapsed().toSec();
    for (auto &point : last_joint_traj_cmd_msg_.points)
    {
        converter << stopwatch_.elapsed().toSec() << delimiter_ << point.time_from_start;
        for (auto &pos : point.positions)
        {
            converter << delimiter_ << pos;
        }
        for (auto &vel : point.velocities)
        {
            converter << delimiter_ << vel;
        }
        converter << "\n";
    }
    return converter.str();
}
*/