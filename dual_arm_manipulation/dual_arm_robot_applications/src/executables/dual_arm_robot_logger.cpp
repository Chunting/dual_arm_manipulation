//
// Created by Chunting on 24.03.17.
//

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "ur_logging/UrLogger.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "dual_arm_robot_logger");
    ros::NodeHandle nh_;
    std::string ur_namespace = nh_.getNamespace();
    std::string prefix;
    ROS_ERROR_STREAM("The UR logger node is created at: " << ur_namespace << "\n");
    ros::param::param("~prefix", prefix, std::string());
    ROS_ERROR_STREAM("The prefix is : " << prefix << "\n");
    
    ros::AsyncSpinner asyncSpinner(4);
    asyncSpinner.start();

    // std::vector<std::string> ur_namespaces;
    // ur_namespaces.push_back("left");
    // ur_namespaces.push_back("right");
    // UR_Logger ur_logger(nh_, ur_namespaces);
    // // sampling rate, hz
    // ur_logger.start(100);
    // // sampling time, second
    // ros::waitForShutdown();
    // ur_logger.stop();

    ros::shutdown();
    return 0;
}