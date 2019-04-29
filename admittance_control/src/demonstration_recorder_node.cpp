#include <iostream>
#include <fstream>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <ros/ros.h>

#include "cartesian_state_msgs/PoseTwist.h"

using namespace std;



class DemoRecorder {

private:
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;

	std::string recPath_;

	std::string prefix_;
	std::string topic_arm_state_;


	//real variables from the robot
	ofstream file_demo_ee_position_;
	ofstream file_demo_ee_orientation_;

	ros::Subscriber sub_ee_state_;




public:
	DemoRecorder(ros::NodeHandle &n, double frequency, std::string prefix, std::string topic_arm_state)
		: nh_(n), loop_rate_(frequency), prefix_(prefix), topic_arm_state_(topic_arm_state) {

		ROS_INFO_STREAM("The recorder node is created at: " << nh_.getNamespace() 
		<< " with freq: " << frequency << "Hz" 
		<< "prefix: " << prefix_
		<< "topic_arm_state_: " << topic_arm_state_);

	}

	void Initialize() {

		// Creating a recording directory
		std::string recPath_ = "/home/Chunting/catkin_ws/src/dual_arm_manipulation/dataLog/";
		mkdir(recPath_.c_str(), 0777);

		// Creating a subdirectory for a specific subject
		recPath_ += "demonstrations/";
		mkdir(recPath_.c_str(), 0777);

		// Creating a subdirectory for a specific interaction based on time
		time_t rawtime;
		tm* timeinfo;
		char buffer [80];
		time(&rawtime);
		timeinfo = localtime(&rawtime);
		strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", timeinfo);
		recPath_ += string(buffer);
		mkdir(recPath_.c_str(), 0777);

		std::cout << "Recording to :" << recPath_.c_str() << endl;

		std::string recPathFile_demo_positions = recPath_ + prefix_+ "_ee_position.csv";
		file_demo_ee_position_.open(recPathFile_demo_positions);
		file_demo_ee_position_  << "Time" << "\t" << "x" << "\t" << "y" << "\t" << "z"
		                        << "\t" << "vx" << "\t" << "vy" << "\t" << "vz" << "\n";

		string recPathFile_demo_orientation = recPath_ + prefix_ + "_ee_orientation.txt";
		file_demo_ee_orientation_.open(recPathFile_demo_orientation);
		file_demo_ee_orientation_  << "Time \t x \t y \t z \t w \t rx \t ry \t rz \n";
        
		sub_ee_state_ = nh_.subscribe(topic_arm_state_ ,
		                              1000, &DemoRecorder::state_arm_callback, this);


	}


	void Run() {

		while (nh_.ok()) {

			ros::spinOnce();
			loop_rate_.sleep();
		}

	}

	void state_arm_callback(
	    const cartesian_state_msgs::PoseTwistConstPtr msg) {

		file_demo_ee_position_  << ros::Time::now()	<< "\t"
		                        << msg->pose.position.x 	<< "\t"
		                        << msg->pose.position.y 	<< "\t"
		                        << msg->pose.position.z 	<< "\t"
		                        << msg->twist.linear.x << "\t"
		                        << msg->twist.linear.y << "\t"
		                        << msg->twist.linear.z << "\n";


		file_demo_ee_orientation_  << ros::Time::now()	<< "\t"
		                           << msg->pose.orientation.x << "\t"
		                           << msg->pose.orientation.y << "\t"
		                           << msg->pose.orientation.z << "\t"
		                           << msg->pose.orientation.w << "\t"
		                           << msg->twist.angular.x << "\t"
		                           << msg->twist.angular.y << "\t"
		                           << msg->twist.angular.z << "\n";

	}

};






int main(int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "demonstration_recorder");

	ros::NodeHandle nh;
	double frequency = 100.0;
	std::string prefix = nh.getNamespace();
	std::string topic_arm_state;

	// std::string nsprefix = name_space+"/prefix";

	ROS_INFO("retrieve the prefix %s in demonstration_recorder_node.", prefix.c_str());

	topic_arm_state = prefix + "/ur5_cartesian_velocity_controller/ee_state";
	DemoRecorder demo_recorder(nh, frequency, prefix, topic_arm_state);
	demo_recorder.Initialize();

	demo_recorder.Run();

	return 0;
}