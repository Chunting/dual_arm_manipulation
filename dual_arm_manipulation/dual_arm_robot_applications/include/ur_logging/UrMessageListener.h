#include <tf/transform_listener.h>
#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

#include "std_msgs/String.h"
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"
#include <sstream>

class UR_Message_Listener
{ //handles callbacks and saves last received messages
  protected:
    ros::NodeHandle nh_;
    ros::Subscriber sub_wrench_external_;
    ros::Subscriber joint_speed_com_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber execTrajectorySub_;
    ros::Subscriber m_tcp_speed_sub_;
    ros::Subscriber c_tcp_speed_sub_;
    ros::Subscriber offset_sub_;


  public:
    UR_Message_Listener(ros::NodeHandle &nh, std::string ur_namespace);

    std::string ur_namespace_;
    sensor_msgs::JointState last_joint_state_msg_;
    geometry_msgs::PoseStamped last_m_tcp_pose_msg_;
    trajectory_msgs::JointTrajectory last_c_joint_vel_msg_;
    geometry_msgs::TwistStamped last_m_tcp_vel_msg_;
    geometry_msgs::TwistStamped last_c_tcp_vel_msg_;
    geometry_msgs::WrenchStamped last_wrench_msg_;
    geometry_msgs::PointStamped last_offset_msg_;

    geometry_msgs::TransformStamped last_tform_msg_;
    moveit_msgs::RobotTrajectory last_trajectory_msg_;

    bool newTrajectory = false;
    std::string ur_prefix_;
    int count = 0;

  private:
    void FT_wrench_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    void c_joint_vel_Callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
    void joint_state_Callback(const sensor_msgs::JointState::Ptr &msg);
    void m_tcp_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void trajectoryCallback(const moveit_msgs::RobotTrajectory::ConstPtr &msg);
    void m_tcp_speedCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void c_tcp_speedCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void offset_Callback(const geometry_msgs::PointStamped::ConstPtr &msg);
};