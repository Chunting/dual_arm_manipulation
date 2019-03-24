//
// Created by Chunting on 22.3.19.
//
// ROS
#ifndef PROJECT_FTSensorSubscriber_H
#define PROJECT_FTSensorSubscriber_H
#include <ros/ros.h>

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

// Robotiq force torque sensor
#include <robotiq_ft_sensor/sensor_accessor.h>
#include "geometry_msgs/WrenchStamped.h"

using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

class FTSensorSubscriber
{ //handles callbacks and saves last received messages
  protected:
	ros::Subscriber sub_wrench_external_;
	ros::Publisher filtered_wrench_pub_;
	ros::NodeHandle nh_;
	std::string ur_namespace_;

	// FORCE/TORQUE-SENSOR FILTER:
	// Parameters for the noisy wrench
	double wrench_filter_factor_;
	double force_dead_zone_thres_;
	double torque_dead_zone_thres_;
	double admittance_ratio_;
	Vector6d wrench_external_;

  public:
	FTSensorSubscriber(ros::NodeHandle &nh, std::string ur_namespace);
	geometry_msgs::WrenchStamped last_wrench_msg_;

  private:
	void wrenchCallback(const geometry_msgs::WrenchStamped::Ptr &msg);
};
#endif
