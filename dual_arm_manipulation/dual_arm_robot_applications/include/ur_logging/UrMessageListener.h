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
  ros::Subscriber sub_cart_state_;
  ros::Subscriber sub_cart_vel_state_;
  ros::Subscriber sub_cart_pose_state_;
  ros::Subscriber sub_cart_vel_cmd_;
  ros::Subscriber sub_cart_pose_cmd_;
  ros::Subscriber sub_wrench_external_;
  ros::Subscriber sub_joint_state_;
  ros::Subscriber sub_joint_traj_cmd_;

  ros::Subscriber sub_robot_traj_cmd_;
  ros::Subscriber sub_offset_pose_state_;
  ros::Subscriber sub_joint_traj_point_cmd_;

  ros::Publisher pub_joint_state_;

public:
  UR_Message_Listener(ros::NodeHandle &nh, std::string ur_namespace, std::string folder_name);
  void write_logfile();

  std::string ur_namespace_;

  moveit_msgs::RobotTrajectory last_robot_traj_cmd_msg_;

  cartesian_state_msgs::PoseTwist   last_cart_state_msg_;
  geometry_msgs::TwistStamped       last_cart_vel_state_msg_;
  geometry_msgs::PoseStamped        last_cart_pose_state_msg_;
  geometry_msgs::Pose               last_offset_pose_state_msg_;

  geometry_msgs::Twist              last_cart_vel_cmd_msg_;
  geometry_msgs::PoseStamped        last_cart_pose_cmd_msg_;

  geometry_msgs::WrenchStamped      last_wrench_msg_;

  sensor_msgs::JointState           last_joint_state_msg_;
  trajectory_msgs::JointTrajectory  last_joint_traj_cmd_msg_;
  trajectory_msgs::JointTrajectoryPoint last_joint_traj_point_cmd_msg_;


  bool newTrajectory = false;
  std::string ur_prefix_;
  int count = 0;

  float wrench_filter_factor_;
  float force_dead_zone_thres_;
  float torque_dead_zone_thres_;
  Vector6d wrench_external_;

  void start(int log_rate);
  void stop();

private:
  std::string topic_cart_state_;
  std::string topic_cart_pose_state_;
  std::string topic_cart_vel_state_;
  std::string topic_cart_vel_cmd_;
  std::string topic_cart_pose_cmd_;
  std::string topic_joint_state_;
  std::string topic_joint_traj_cmd_;
  std::string topic_external_wrench;
  std::string topic_robot_traj_cmd_;
  std::string topic_offset_pose_state_;
  std::string topic_joint_traj_point_cmd_;

  // real variables from the robot
  // std::ofstream file_cartesian_state_;
  std::ofstream file_robot_state_;
  std::ofstream file_robot_cmd_;
  // std::ofstream file_cart_vel_state_;
  // std::ofstream file_cart_vel_cmd_;
  // std::ofstream file_cart_pose_cmd_;
  // std::ofstream file_wrench_;
  // std::ofstream file_joint_state_;
  // std::ofstream file_joint_traj_cmd_;

  char delimiter_;
  std::string folder_name_;
  Stopwatch stopwatch_;
  double start_time_ = 0.0;
  double pre_cmd_time_ =0.0;
  ros::Timer timer_;


  void cart_state_callback(const cartesian_state_msgs::PoseTwist::ConstPtr &msg);
  void cart_pose_state_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void cart_vel_state_callback(const geometry_msgs::TwistStamped::ConstPtr &msg);
  void cart_vel_cmd_callback(const geometry_msgs::Twist::ConstPtr &msg);
  void cart_pose_cmd_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void wrench_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
  void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);
  void joint_traj_cmd_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
  void joint_traj_point_cmd_callback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &msg);

  bool generate_logfile();       //automatically generate a file name
  bool waitForValid(double seconds = 15); // It takes a little bit long to connect to ft sensor

  void robot_traj_cmd_callback(const moveit_msgs::RobotTrajectory::ConstPtr &msg);
  void offset_pose_state_callback(const geometry_msgs::Pose::ConstPtr &msg);

  void logCallback(const ros::TimerEvent&);
};
#endif