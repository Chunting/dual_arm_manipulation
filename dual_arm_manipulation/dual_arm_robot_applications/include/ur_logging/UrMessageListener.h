#ifndef PROJECT_URMESSAGELISTENER_H
#define PROJECT_URMESSAGELISTENER_H
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include "cartesian_state_msgs/PoseTwist.h"
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <sstream>
#include <fstream>
#include <sys/stat.h>
#include <time.h>
#include <locale>
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"
#include "std_msgs/String.h"

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "ur_logging/Stopwatch.h"

using namespace Eigen;
typedef Matrix<double, 6, 1> Vector6d;

class UR_Message_Listener
{  // handles callbacks and saves last received messages
protected:
  ros::NodeHandle nh_;
  ros::Subscriber sub_cartesian_state_;
  ros::Subscriber sub_cartesian_vel_state_;
  ros::Subscriber sub_cartesian_pos_state_;
  ros::Subscriber sub_cartesian_vel_cmd_;
  ros::Subscriber sub_cartesian_pos_cmd_;
  ros::Subscriber sub_wrench_external_;
  ros::Subscriber sub_joint_state_;
  ros::Subscriber sub_joint_traj_cmd_;

  ros::Subscriber joint_speed_com_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber execTrajectorySub_;
  ros::Subscriber m_tcp_speed_sub_;
  ros::Subscriber c_tcp_speed_sub_;
  ros::Subscriber offset_sub_;

public:
  UR_Message_Listener(ros::NodeHandle &nh, std::string ur_namespace, std::string folder_name, double frequency=100);

  std::string ur_namespace_;
 
  geometry_msgs::PoseStamped last_m_tcp_pose_msg_;
  trajectory_msgs::JointTrajectory last_c_joint_vel_msg_;
  geometry_msgs::TwistStamped last_m_tcp_vel_msg_;
  geometry_msgs::TwistStamped last_c_tcp_vel_msg_;
  
  geometry_msgs::PointStamped last_offset_msg_;

  geometry_msgs::TransformStamped last_tform_msg_;
  moveit_msgs::RobotTrajectory last_trajectory_msg_;

  cartesian_state_msgs::PoseTwist last_cartesian_state_msg_;
  geometry_msgs::TwistStamped last_cartesian_vel_state_msg_;
  geometry_msgs::PoseStamped last_cartesian_pos_state_msg_;
  geometry_msgs::Twist last_cartesian_vel_cmd_msg_;
  geometry_msgs::PoseStamped last_cartesian_pos_cmd_msg_;
  geometry_msgs::WrenchStamped last_wrench_msg_;
  sensor_msgs::JointState last_joint_state_msg_;
  trajectory_msgs::JointTrajectory last_joint_traj_cmd_msg_;


  bool newTrajectory = false;
  std::string ur_prefix_;
  int count = 0;

  float wrench_filter_factor_;
  float force_dead_zone_thres_;
  float torque_dead_zone_thres_;
  Vector6d wrench_external_;

private:
  std::string topic_cartesian_state_;
  std::string topic_cartesian_pos_state_;
  std::string topic_cartesian_vel_state_;
  std::string topic_joint_state_;
  std::string topic_cartesian_vel_cmd_;
  std::string topic_cartesian_pos_cmd_;
  std::string topic_joint_traj_cmd_;
  std::string topic_external_wrench;

  // real variables from the robot
  std::ofstream file_cartesian_state_;
  std::ofstream file_cartesian_pos_state_;
  std::ofstream file_cartesian_vel_state_;
  std::ofstream file_cartesian_vel_cmd_;
  std::ofstream file_cartesian_pos_cmd_;
  std::ofstream file_wrench_;
  std::ofstream file_joint_state_;
  std::ofstream file_joint_cmd_;

  ros::Rate loop_rate_;
  char delimiter_;
  std::string folder_name_;
  Stopwatch stopwatch_;


  void cartesian_state_callback(const cartesian_state_msgs::PoseTwistConstPtr &msg);
  void cartesian_pos_state_callback(const geometry_msgs::PoseStampedPtr &msg);
  void cartesian_vel_state_callback(const geometry_msgs::TwistStampedConstPtr &msg);
  void cartesian_vel_cmd_callback(const geometry_msgs::TwistConstPtr &msg);
  void cartesian_pos_cmd_callback(const geometry_msgs::PoseStampedPtr &msg);
  void wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg);
  void joint_state_callback(const sensor_msgs::JointState::Ptr &msg);
  void joint_traj_cmd_callback(const trajectory_msgs::JointTrajectoryConstPtr &msg);

  void generate_logfile();       //automatically generate a file name
 

  void FT_wrench_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
  void c_joint_vel_Callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
 
  void m_tcp_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void trajectoryCallback(const moveit_msgs::RobotTrajectory::ConstPtr &msg);
  void m_tcp_speedCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
  void c_tcp_speedCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
  void offset_Callback(const geometry_msgs::PointStamped::ConstPtr &msg);
};
#endif