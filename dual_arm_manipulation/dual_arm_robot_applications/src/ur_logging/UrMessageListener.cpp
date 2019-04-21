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

    ros::ServiceClient client = nh_.serviceClient<robotiq_ft_sensor::sensor_accessor>(ur_namespace +"/robotiq_ft_sensor_acc");

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
    last_wrench_msg_ = *msg;
    std::string topic = ur_namespace_ + "/robotiq_ft_wrench";
    // ROS_INFO("I received last_wrench_msg_ to [%s]: Frame_id [%s] Time [%f] FX[%f] FY[%f] FZ[%f] MX[%f] MY[%f] MZ[%f]",
    //     topic.c_str(),
    //     last_wrench_msg_.header.frame_id.c_str(),   // robotiq_ft_frame_id
    //     last_wrench_msg_.header.stamp.toSec(),
    //     last_wrench_msg_.wrench.force.x, last_wrench_msg_.wrench.force.y, last_wrench_msg_.wrench.force.z,
    //     last_wrench_msg_.wrench.torque.x, last_wrench_msg_.wrench.torque.y, last_wrench_msg_.wrench.torque.z);
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