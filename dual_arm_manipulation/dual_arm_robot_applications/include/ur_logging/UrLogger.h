#ifndef PROJECT_URLOGGER_H
#define PROJECT_URLOGGER_H
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <typeinfo>
#include <time.h>

#include <iomanip>
#include <locale>

#include "ur_logging/Stopwatch.h"
#include "ur_logging/UrMessageListener.h"

class UR_Logger{    //Logs all important messages from ur. It can handle several robots at a time
protected:
    ros::NodeHandle nh_;
    std::vector<std::shared_ptr<UR_Message_Listener>> ur_listeners_;

public:
    UR_Logger(ros::NodeHandle& nh, std::vector<std::string> &ur_namespaces);
    ~UR_Logger();

    Stopwatch stopwatch_;
    std::string ft_sensor_logfile_name_;
    std::string logfolder_name_ = "/home/chunting/catkin_ws/src/dual_arm_manipulation/dataLog/";
    std::string robot_cart_pose_logfile_name_;
    char delimiter_;

    void generate_logfile_name();      
    std::string generate_logfolder();
    void start(int log_rate = 100);     
    void stop();
    bool create_ft_sensor_logfile();
    bool create_robot_cart_pose_logfile();
    std::string ft_sensor_headline(UR_Message_Listener &ur_listener);
    std::string ft_sensor_headline();
    std::string ft_sensor_dataline(UR_Message_Listener &ur_listener);
    std::string ft_sensor_dataline();
    std::string robot_cart_pose_headline(UR_Message_Listener &ur_listener);
    std::string robot_cart_pose_headline();
    std::string robot_cart_pose_dataline(UR_Message_Listener &ur_listener);
    std::string robot_cart_pose_dataline();

    std::string headline(UR_Message_Listener &ur_listener);	//return a headline
    std::string data_line(UR_Message_Listener &ur_listener);	//return formattet data
    std::string headline_command(UR_Message_Listener &ur_listener);	//return a headline
    std::string data_line_command(UR_Message_Listener &ur_listener);
    int command_count = 0;
private:
    ros::Timer timer_;
    std::ofstream ft_sensor_logfile_stream_;
    std::ofstream robot_cart_pose_logfile_stream_;
    ros::Time log_start_time_;

    void logCallback(const ros::TimerEvent&);
};
#endif
