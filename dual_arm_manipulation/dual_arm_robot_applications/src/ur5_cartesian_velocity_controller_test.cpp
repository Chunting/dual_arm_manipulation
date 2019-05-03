#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/SwitchControllerRequest.h>
#include <controller_manager_msgs/SwitchControllerResponse.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "cartesian_state_msgs/PoseTwist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "dual_arm_toolbox/Transform.h"

// rosrun dual_arm_robot_applications ur5_cartesian_velocity_controller_test
using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

// Arm state: position, orientation, and twist (in "base_link")
Vector3d arm_real_position_;
Quaterniond arm_real_orientation_;
Vector6d arm_real_twist_;

///////////////////////////////////////////////////////////////
////////////////////////// Callbacks //////////////////////////
///////////////////////////////////////////////////////////////
void state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg)
{
  arm_real_position_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  arm_real_orientation_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
	  msg->pose.orientation.w;

  arm_real_twist_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z, msg->twist.angular.x,
	  msg->twist.angular.y, msg->twist.angular.z;
  // std::cout << "arm_real_position_" << std::endl
  // 		  << arm_real_position_ << std::endl
  // 		  << "arm_real_orientation_" << std::endl
  // 		  << arm_real_orientation_.coeffs() << std::endl
  // 		  << "arm_real_twist_" << std::endl
  // 		  << arm_real_twist_ << std::endl;
}
bool switch_controller(ros::NodeHandle &nh_, std::string stop_name, std::string start_name, std::string ur_namespace)
{
  ROS_INFO("Switching controllers");
  // setup
  ros::ServiceClient srv_switch_controller = nh_.serviceClient<controller_manager_msgs::SwitchController>(
	  ur_namespace + "/controller_manager/switch_controller");
  controller_manager_msgs::SwitchController switchController;
  switchController.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
  sleep(2);

  // stop
  switchController.request.stop_controllers.push_back(stop_name);
  bool success_stop = srv_switch_controller.call(switchController);
  ROS_INFO("Stopping controller %s", success_stop ? "SUCCEDED" : "FAILED");
  if (!success_stop)
	return false;

  // clear
  switchController.request.stop_controllers.clear();

  // start admittance controller
  switchController.request.BEST_EFFORT;
  switchController.request.start_controllers.push_back(start_name);
  bool success_start = srv_switch_controller.call(switchController);
  ROS_INFO("Starting controller %s", success_start ? "SUCCEDED" : "FAILED");
  switchController.request.start_controllers.clear();

  return success_start;
}

bool get_rotation_matrix(Matrix6d &rotation_matrix, tf::TransformListener &listener, std::string from_frame,
						 std::string to_frame)
{
  tf::StampedTransform transform;
  Matrix3d rotation_from_to;
  try
  {
	listener.lookupTransform(from_frame, to_frame, ros::Time(0), transform);
	tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
	rotation_matrix.setZero();
	rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
	rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
	std::cout << "Get TF from" << from_frame << " to: " << to_frame << std::endl
			  << rotation_from_to << std::endl
			  << "rotation_from_to" << std::endl
			  << rotation_from_to << std::endl;
  }
  catch (tf::TransformException ex)
  {
	rotation_matrix.setZero();
	ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame);
	return false;
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cartesian_speed_talker");

  ros::NodeHandle nh_;
  ros::Rate loop_rate(100);
  tf::TransformListener listener_arm_;
  if (!switch_controller(nh_, "vel_based_pos_traj_controller", "ur5_cartesian_velocity_controller", "left"))
	ROS_WARN("Failed switching left controller");
  if (!switch_controller(nh_, "vel_based_pos_traj_controller", "ur5_cartesian_velocity_controller", "right"))
	ROS_WARN("Failed switching right controller");
  sleep(3);

  std::string topic_right_arm_state("/right/ur5_cartesian_velocity_controller/ee_state");
  std::string topic_right_arm_cmd("/right/ur5_cartesian_velocity_controller/command_cart_vel");
  ros::Subscriber sub_right_arm_state_ = nh_.subscribe(topic_right_arm_state, 10, &state_arm_callback);
  ros::Publisher pub_right_arm_cmd_ = nh_.advertise<geometry_msgs::Twist>(topic_right_arm_cmd, 5);

  Matrix6d rotation_right_base_world;
  Vector6d world_right_arm_cmd_twist_;
  Vector6d right_base_arm_cmd_twist_;
  world_right_arm_cmd_twist_ << 0, 0, 0, 0.01, 0, 0;
  while (!get_rotation_matrix(rotation_right_base_world, listener_arm_, "right_base_link", "world"))
  {
	sleep(1);
  }
  right_base_arm_cmd_twist_ = rotation_right_base_world * world_right_arm_cmd_twist_;

  std::string topic_left_arm_state("/left/ur5_cartesian_velocity_controller/ee_state");
  std::string topic_left_arm_cmd("/left/ur5_cartesian_velocity_controller/command_cart_vel");
  ros::Subscriber sub_left_arm_state_ = nh_.subscribe(topic_left_arm_state, 10, &state_arm_callback);
  ros::Publisher pub_left_arm_cmd_ = nh_.advertise<geometry_msgs::Twist>(topic_left_arm_cmd, 5);

  Vector6d world_left_arm_cmd_twist_;
  Matrix6d rotation_left_base_world;
  Vector6d left_base_arm_cmd_twist_;
  world_left_arm_cmd_twist_ << 0, 0, 0, 0.01, 0, 0;
  while (!get_rotation_matrix(rotation_left_base_world, listener_arm_, "left_base_link", "world"))
  {
	sleep(1);
  }
  left_base_arm_cmd_twist_ = rotation_left_base_world * world_left_arm_cmd_twist_;
  /*
  rotation_right_base_world
  0.000796327  0           -1
  0.707669    -0.706543  0.000563536
 -0.706543     -0.70767 -0.000562639


  chunting@chunting-Vostro-3670:~/catkin_ws$ rosrun tf tf_echo  left_base_link world
  At time 0.000
  - Translation: [1.200, -0.177, -0.176]
  - Rotation: in Quaternion [0.653, 0.271, -0.653, -0.271]
		  in RPY (radian) [-1.572, 0.785, 1.570]
		  in RPY (degree) [-90.046, 44.954, 89.936]
  */

  // std::cout << "world_right_arm_cmd_twist_" << std::endl
  // 		  << world_right_arm_cmd_twist_ << std::endl
  // 		  << "right_base_arm_cmd_twist_" << std::endl
  // 		  << right_base_arm_cmd_twist_ << std::endl
  // 		  << "rotation_right_base_world" << std::endl
  // 		  << rotation_right_base_world << std::endl;
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // for the arm
  geometry_msgs::Twist right_arm_twist_cmd;
  geometry_msgs::Twist left_arm_twist_cmd;
  int direction = -1;
  double cart_speed = 0.01;
  dual_arm_toolbox::Transform::transformVector6dtoTwist(left_base_arm_cmd_twist_, left_arm_twist_cmd);
  dual_arm_toolbox::Transform::transformVector6dtoTwist(right_base_arm_cmd_twist_, right_arm_twist_cmd);
  while (nh_.ok())
  {
	pub_right_arm_cmd_.publish(right_arm_twist_cmd);
	pub_left_arm_cmd_.publish(left_arm_twist_cmd);
	ros::spinOnce();
	loop_rate.sleep();
	direction = -direction;
  }
  ros::shutdown();

  return 0;
}