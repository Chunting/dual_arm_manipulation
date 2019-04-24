#include "ur_logging/UrMessageListener.h"

//UR_Message_Listener::UR_Message_Listener(ros::NodeHandle& nh, std::string ur_namespace) : nh_(nh), ur_namespace_(ur_namespace){
UR_Message_Listener::UR_Message_Listener(ros::NodeHandle &nh, std::string ur_namespace) : nh_(nh), ur_namespace_(ur_namespace)
{
    if (ur_namespace_.size() > 0)
    {
        ur_prefix_ = ur_namespace_ + "_";
    }
    // Measured joint variables
    state_sub_ = nh_.subscribe("/joint_states", 1, &UR_Message_Listener::joint_state_Callback, this);
    // It takes a long time to get message from this topic, about 0.5s

    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(ur_namespace_ + "/pose", 10, &UR_Message_Listener::m_tcp_pose_Callback, this);

    sub_wrench_external_ = nh_.subscribe<geometry_msgs::WrenchStamped>(ur_namespace_ + "/robotiq_ft_wrench", 1, &UR_Message_Listener::FT_wrench_Callback, this);
    // Measured tcp velocity
    m_tcp_speed_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(ur_namespace + "/m_tool_velocity", 10, &UR_Message_Listener::m_tcp_speedCallback, this);

    // Joint speed command
    joint_speed_com_sub_ = nh_.subscribe<trajectory_msgs::JointTrajectory>(ur_namespace + "/ur_driver/joint_speed", 10, &UR_Message_Listener::c_joint_vel_Callback, this);

    c_tcp_speed_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(ur_namespace + "/c_tool_velocity", 10, &UR_Message_Listener::c_tcp_speedCallback, this);

    execTrajectorySub_ = nh_.subscribe<moveit_msgs::RobotTrajectory>("/execute_my_trajectory", 10, &UR_Message_Listener::trajectoryCallback, this);

    offset_sub_ = nh_.subscribe("/offset_point", 1, &UR_Message_Listener::offset_Callback, this);

    newTrajectory = false;

    ros::ServiceClient client = nh_.serviceClient<robotiq_ft_sensor::sensor_accessor>(ur_namespace + "/robotiq_ft_sensor_acc");

    robotiq_ft_sensor::sensor_accessor srv;

    int count = 0;
    if (ros::ok())
    {
        srv.request.command = "SET ZRO";
        if (client.call(srv))
        {
            ROS_INFO("ret: %s", srv.response.res.c_str());
        }
    }
    wrench_external_.setZero();
    wrench_filter_factor_ = 0.1;
    force_dead_zone_thres_ = 5;
    torque_dead_zone_thres_ = 0.5;
}

void UR_Message_Listener::c_joint_vel_Callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
    last_c_joint_vel_msg_ = *msg;
}

void UR_Message_Listener::joint_state_Callback(const sensor_msgs::JointState::Ptr &msg)
{
    std::string name = (*msg).name[0];
    // "joint_states" receives messages of all robots. This is seperating the searched one from the others.
    if (name.compare(0, ur_prefix_.size(), ur_prefix_) == 0)
    {
        last_joint_state_msg_ = *msg;
    }
}

// Be careful that it can't compile with const trajectory_msgs::JointTrajectory::Ptr
void UR_Message_Listener::FT_wrench_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    // last_wrench_msg_ = *msg;
    Vector6d wrench_ft_frame;
    // Reading the FT-sensor in its own frame (robotiq_ft_frame_id)
    wrench_ft_frame << msg->wrench.force.x, msg->wrench.force.y,
        msg->wrench.force.z, msg->wrench.torque.x,
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
    wrench_external_ = (1 - wrench_filter_factor_) * wrench_external_ +
                       wrench_filter_factor_ * wrench_ft_frame;
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
        //     ROS_INFO("Listening Points %d  %f ", i, last_trajectory_msg_.joint_trajectory.header.stamp.toSec()+last_trajectory_msg_.joint_trajectory.points[i].time_from_start.toSec());
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