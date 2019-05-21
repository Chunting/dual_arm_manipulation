//
// Created by Chunting on 24.03.17.
//

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "ur_logging/UrMessageListener.h"
#include "ur_logging/UrLogger.h"

std::string generate_logfolder(){
    // Creating a recording directory
  std::string recPath_ = "/home/chunting/catkin_ws/src/dual_arm_manipulation/dataLog/";
  time_t rawtime;
  struct tm *timeinfo;
  char buffer[20];

  time(&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer, 80, "%Y_%m_%d_%H_%M", timeinfo);
  recPath_ = recPath_ + std::string(buffer) + '/';
  const int folder = mkdir(recPath_.c_str(), 0777);
  if(folder == -1){
      ROS_ERROR("Failed to creat log folder %s", recPath_.c_str());
  }
  return recPath_;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "dual_arm_robot_logger");
    ros::NodeHandle nh_;
    // // std::string ur_namespace = nh_.getNamespace();  // "//left" is wired, should be "left" or "/left"
    // std::string prefix;
    // ros::param::param("~prefix", prefix, std::string());
    // ROS_INFO_STREAM("The prefix is : " << prefix << "\n");
    
    // ros::AsyncSpinner asyncSpinner(4);
    // asyncSpinner.start();
    // std::string logfolder_name_ = generate_logfolder();
    // std::shared_ptr<UR_Message_Listener> ur_listenerPtr 
    //     = std::make_shared<UR_Message_Listener> (nh_, prefix, logfolder_name_);
    // ur_listenerPtr->start(100);
    // ros::waitForShutdown();
    // ur_listenerPtr->stop();
   
    std::vector<std::string> ur_namespaces;
    ur_namespaces.push_back("left");
    ur_namespaces.push_back("right");
    UR_Logger ur_logger(nh_, ur_namespaces);
    // sampling rate, hz
    ur_logger.start(100);
    // sampling time, second
    ros::waitForShutdown();
    ur_logger.stop();
    
    ros::shutdown();
    return 0;
}