//
// Created by Chunting  on 07.10.16.
// rosrun dual_arm_robot_applications dual_arm_robot_demonstration

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>

// Rviz
#include <moveit_msgs/DisplayTrajectory.h>

// Dual Arm Tools
#include "dual_arm_toolbox/TrajectoryProcessor.h"
#include "dual_arm_toolbox/Transform.h"

// Dual Arm Demonstrator
#include "dual_arm_demonstrator_iml/DualArmRobot.h"
#include "dual_arm_demonstrator_iml/SceneManager.h"

//UrLogger
#include "ur_logging/UrLogger.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_arm_robot_demonstration");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle nh;
    // Dual Arm Robot Setup
    dual_arm_demonstrator_iml::DualArmRobot dualArmRobot(nh);
    dual_arm_demonstrator_iml::SceneManager sceneManager(nh);
    sceneManager.setupScene();
    ROS_INFO("========== MOVE HOME POSITION =================");
    dualArmRobot.moveHome();
    
    ros::shutdown();
    return 0;
}