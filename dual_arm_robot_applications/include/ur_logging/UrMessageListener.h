#include <tf/transform_listener.h>
#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

#include "std_msgs/String.h"
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"
#include <sstream>

class UR_Message_Listener{  //handles callbacks and saves last received messages
protected:
    ros::NodeHandle nh_;
    ros::Subscriber wrench_sub_;
    ros::Subscriber speed_traj_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber execTrajectorySub_;


public:
    UR_Message_Listener(ros::NodeHandle& nh, std::string ur_namespace);

    std::string ur_namespace_;
    trajectory_msgs::JointTrajectory last_speed_traj_msg_;
    geometry_msgs::WrenchStamped last_wrench_msg_;
    sensor_msgs::JointState last_state_msg_;
    geometry_msgs::TransformStamped last_tform_msg_;
    geometry_msgs::PoseStamped last_pose_msg_;
    moveit_msgs::RobotTrajectory last_trajectory_msg_;
    robotiq_ft_sensor::ft_sensor last_force_msg_;
    bool newTrajectory = false;
    std::string ur_prefix_;
    int count = 0;
    
private:
    void reCallback(const robotiq_ft_sensor::ft_sensor::Ptr& msg);
    void wrenchCallback(const geometry_msgs::WrenchStamped::Ptr& msg);
    void speed_trajCallback(const trajectory_msgs::JointTrajectory::Ptr& msg);
    void stateCallback(const sensor_msgs::JointState::Ptr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void trajectoryCallback(const moveit_msgs::RobotTrajectory::ConstPtr& msg);
};
