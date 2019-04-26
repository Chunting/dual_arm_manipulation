#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/SwitchControllerRequest.h>
#include <controller_manager_msgs/SwitchControllerResponse.h>

#include "cartesian_state_msgs/PoseTwist.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"
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
void state_arm_callback(
	const cartesian_state_msgs::PoseTwistConstPtr msg)
{
	arm_real_position_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
	arm_real_orientation_.coeffs() << msg->pose.orientation.x,
		msg->pose.orientation.y,
		msg->pose.orientation.z,
		msg->pose.orientation.w;

	arm_real_twist_ << msg->twist.linear.x, msg->twist.linear.y,
		msg->twist.linear.z, msg->twist.angular.x, msg->twist.angular.y,
		msg->twist.angular.z;
	std::cout << arm_real_position_ << std::endl;
}
int main(int argc, char **argv)
{
	bool first = true;
	ros::init(argc, argv, "joint_speed_talker");
	ros::NodeHandle nh_;
	std::string TOPIC_ARM_STATE("/left/ur5_cartesian_velocity_controller/ee_state");
	std::string TOPIC_ARM_COMMAND("/left/ur5_cartesian_velocity_controller/command_cart_vel");
	// Subscribers
	ros::Subscriber sub_arm_state_ = nh_.subscribe(TOPIC_ARM_STATE, 10, &state_arm_callback);
	// Publishers
	ros::Publisher pub_arm_cmd_ = nh_.advertise<geometry_msgs::Twist>(TOPIC_ARM_COMMAND, 5);

	ros::Rate loop_rate(100);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// // stop controller
	// ros::ServiceClient left_srv_switch_controller = nh_.serviceClient<controller_manager_msgs::SwitchController>("/left/controller_manager/switch_controller");
	// controller_manager_msgs::SwitchController switch_controller;
	// switch_controller.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
	// switch_controller.request.stop_controllers.push_back("/left/vel_based_pos_traj_controller");
	// bool success = left_srv_switch_controller.call(switch_controller);
	// ROS_INFO("Stopping controller %s", success ? "SUCCEDED" : "FAILED");
	// if (!success)
	// 	return 0;
	// switch_controller.request.stop_controllers.clear();

	// // clear
	// switch_controller.request.stop_controllers.clear();

	// // start admittance controller
	// switch_controller.request.BEST_EFFORT;
	// switch_controller.request.start_controllers.push_back("/left/ur5_cartesian_velocity_controller");
	// bool success_start = left_srv_switch_controller.call(switch_controller);
	// ROS_INFO("Starting controller %s", success_start ? "SUCCEDED" : "FAILED");
	// switch_controller.request.start_controllers.clear();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // for the arm
	geometry_msgs::Twist arm_twist_cmd;
	while (nh_.ok())
	{
		arm_twist_cmd.linear.x = -0.1;
		arm_twist_cmd.linear.y = 0;
		arm_twist_cmd.linear.z = 0;
		arm_twist_cmd.angular.x = 0;
		arm_twist_cmd.angular.y = 0;
		arm_twist_cmd.angular.z = 0;
		pub_arm_cmd_.publish(arm_twist_cmd);
		ros::spinOnce();
		loop_rate.sleep();
	}
}