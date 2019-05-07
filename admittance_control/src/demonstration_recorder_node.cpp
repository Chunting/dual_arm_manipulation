#include <ros/ros.h>
#include <sys/stat.h>
#include <time.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <sstream>
#include <typeinfo>
#include "cartesian_state_msgs/PoseTwist.h"

using namespace std;

class DemoRecorder
{
private:
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;

  std::string recPath_;

  std::string prefix_;
  std::string topic_arm_state_;

  char delimiter_;

  // real variables from the robot
  std::ofstream file_demo_ee_pose_;
  std::ofstream file_demo_ee_velocity_;

  ros::Subscriber sub_ee_state_;

public:
  DemoRecorder(ros::NodeHandle &n, double frequency, std::string prefix, std::string topic_arm_state)
	: nh_(n), loop_rate_(frequency), prefix_(prefix), topic_arm_state_(topic_arm_state)
  {
	delimiter_ = ',';
	ROS_INFO_STREAM("The recorder node is created at: " << nh_.getNamespace() << " with freq: " << frequency
														<< "Hz prefix: " << prefix_
														<< "  topic_arm_state_: " << topic_arm_state_);
  }

  void Initialize()
  {
	// Creating a recording directory
	std::string recPath_ = "/home/chunting/catkin_ws/src/dual_arm_manipulation/dataLog/";

	// Creating a subdirectory for a specific interaction based on time
	time_t rawtime;
	struct tm *timeinfo;
	char buffer[80];
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", timeinfo);
	recPath_ = recPath_ + std::string(buffer) + '/';
	const int dir_err = mkdir(recPath_.c_str(), 0777);
	sub_ee_state_ =
		nh_.subscribe("/left/ur5_cartesian_velocity_controller/ee_state", 10, &DemoRecorder::state_arm_callback, this);

	std::string recPathFile_demo_positions = recPath_ + prefix_ + "_ee_position.csv";
	ROS_INFO_STREAM("recPathfile_demo : " << recPathFile_demo_positions);
	file_demo_ee_pose_.open(recPathFile_demo_positions.c_str(), std::ofstream::out | std::ofstream::trunc);
	if (!file_demo_ee_pose_.is_open())
	{
	  ROS_ERROR("Failed to open %s", recPathFile_demo_positions.c_str());
	}

	file_demo_ee_pose_ << "Time" << delimiter_ << "x" << delimiter_ << "y" << delimiter_ << "z" << delimiter_ << "vx"
					   << delimiter_ << "vy" << delimiter_ << "vz"
					   << "\n";

	std::string recPathFile_demo_orientation = recPath_ + prefix_ + "_ee_orientation.csv";
	file_demo_ee_velocity_.open(recPathFile_demo_orientation);
	if (!file_demo_ee_velocity_.is_open())
	{
	  ROS_ERROR("Recording %s failed", recPathFile_demo_orientation.c_str());
	}

	file_demo_ee_velocity_ << "Time \t x \t y \t z \t w \t rx \t ry \t rz \n";
  }

  void Run()
  {
	while (nh_.ok())
	{
	  ros::spinOnce();
	  loop_rate_.sleep();
	}
  }

  void state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg)
  {
	file_demo_ee_pose_ << ros::Time::now() << delimiter_ << msg->pose.position.x << delimiter_ << msg->pose.position.y
					   << delimiter_ << msg->pose.position.z << delimiter_ << msg->pose.orientation.x << delimiter_
					   << msg->pose.orientation.y << delimiter_ << msg->pose.orientation.z << delimiter_
					   << msg->pose.orientation.w << "\n";

	file_demo_ee_velocity_ << ros::Time::now() << delimiter_ << msg->twist.linear.x << delimiter_ << msg->twist.linear.y
						   << delimiter_ << msg->twist.linear.z << delimiter_ << msg->twist.angular.x << delimiter_
						   << msg->twist.angular.y << delimiter_ << msg->twist.angular.z << "\n";
  }
};

int main(int argc, char **argv)
{
  // Initiate ROS
  ros::init(argc, argv, "demonstration_recorder");

  ros::NodeHandle nh;
  ros::AsyncSpinner asyncSpinner(2);
  asyncSpinner.start();
  double frequency = 100.0;
  std::string prefix;
  std::string topic_arm_state;

  if (!nh.getParam("prefix", prefix))
  {
	ROS_ERROR("Couldn't retrieve the prefix.");
	return -1;
  }

  // std::string nsprefix = name_space+"/prefix";
  topic_arm_state = prefix + "/ur5_cartesian_velocity_controller/ee_state";
  ROS_INFO_STREAM("In demonstration_recorder_node, retrieve the prefix :" << prefix << "  topic : " << topic_arm_state);

  DemoRecorder demo_recorder(nh, frequency, prefix, topic_arm_state);
  demo_recorder.Initialize();

  demo_recorder.Run();
  ros::shutdown();
  return 0;
}