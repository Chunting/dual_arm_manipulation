#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/SwitchControllerRequest.h>
#include <controller_manager_msgs/SwitchControllerResponse.h>

int main(int argc, char **argv)
{
	bool first = true;
	ros::init(argc, argv, "joint_speed_talker");
	ros::NodeHandle nh;
	ros::Publisher chatter_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/left/ur_driver/joint_speed", 1000);

	ros::Rate loop_rate(100);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// stop controller
	ros::ServiceClient left_srv_switch_controller = nh.serviceClient<controller_manager_msgs::SwitchController>("/left/controller_manager/switch_controller");
	controller_manager_msgs::SwitchController srv_req;
	srv_req.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
	srv_req.request.stop_controllers.push_back("/left/left_vel_based_pos_traj_controller");
	bool success = left_srv_switch_controller.call(srv_req);
	ROS_INFO("Stopping controller %s", success ? "SUCCEDED" : "FAILED");
	if (!success)
		return 0;
	srv_req.request.stop_controllers.clear();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	sleep(0.5);
	trajectory_msgs::JointTrajectory trj;
	trajectory_msgs::JointTrajectoryPoint trjp;
	trjp.velocities.push_back(0.0);
	trjp.velocities.push_back(0.0);
	trjp.velocities.push_back(0.0);
	trjp.velocities.push_back(0.0);
	trjp.velocities.push_back(0.0);
	trjp.velocities.push_back(0.0);
	trj.points.push_back(trjp);
	ros::Time be = ros::Time::now();
	while (ros::ok())
	{
		trj.header.stamp = ros::Time::now();
		ros::Time k = ros::Time::now();
		if (first)
		{
			first = false;
			be = k;
		}
		ros::Duration du = k - be;
		std_msgs::Float64 vel;
		vel.data = 0;
		// vel.data = 0.5*sin(du.toSec());
		// trj.points[0].velocities[0] = vel.data;

		//vel.data = 0.15*sin(du.toSec());
		//trj.points[0].velocities[1] = vel.data;

		//vel.data = 0.25*sin(du.toSec());
		//trj.points[0].velocities[2] = vel.data;

		//vel.data = 0.35*sin(du.toSec());
		//trj.points[0].velocities[3] = vel.data;

		//vel.data = 0.15*sin(du.toSec());
		//trj.points[0].velocities[4] = vel.data;

		vel.data = 0.15 * sin(du.toSec());
		trj.points[0].velocities[5] = vel.data;
		chatter_pub.publish(trj);
		if (du.toSec() > 18.8495556 * 10)
			break;
		ros::spinOnce();
		loop_rate.sleep();
	}
	trj.points[0].velocities[0] = 0;
	trj.header.stamp = ros::Time::now();
	chatter_pub.publish(trj);
}