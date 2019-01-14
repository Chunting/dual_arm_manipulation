// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group.h>
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

// UR Logger
#include "ur_logging/UrLogger.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "shelf_setup_helper");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle nh;

    // Dual Arm Robot Setup
    dual_arm_demonstrator_iml::DualArmRobot dualArmRobot(nh);

    // Scene Setup
    dual_arm_demonstrator_iml::SceneManager sceneManager(nh);
    sceneManager.setupScene();

    // variables
    moveit::planning_interface::MoveGroup::Plan left_plan;
    moveit::planning_interface::MoveGroup::Plan right_plan;
    moveit::planning_interface::MoveItErrorCode error;
    error.val = -1;
   
    geometry_msgs::Vector3Stamped direction;
    direction.header.frame_id = "world";
   

    std::vector<std::string> ur_namespaces;
    ur_namespaces.push_back("left");
    ur_namespaces.push_back("right");
    UR_Logger ur_logger(nh, ur_namespaces);
    ur_logger.start(125);

    ROS_INFO("========== MOVE HOME POSITION =================");
    dualArmRobot.moveHome();
    sleep(1);
    ROS_INFO("========== MOVE GRASP POSITION =================");
    dualArmRobot.moveGraspPosition();
    sleep(1);
    ROS_INFO("========== MOVE CLOSER =================");
    dualArmRobot.graspMove(0.01);
    // sleep(1);
    // ROS_INFO("========== GET OFFSET =================");
    // dualArmRobot.getCurrentOffset();
    // sleep(1);
    // ROS_INFO("========== LIFT BOX BACK =================");
    // direction.vector.x = 0;
    // direction.vector.y = 0;
    // direction.vector.z = 0.1;
    // dualArmRobot.linearMove(direction, false, true,true);
    // ROS_INFO("========== LEFT MOVE =================");
    // direction.vector.x = 0;
    // direction.vector.y = -0.2;
    // direction.vector.z = 0;
    // dualArmRobot.linearMove(direction, true, true,true);
    // ROS_INFO("========== RIGHT MOVE =================");
    // direction.vector.x = 0;
    // direction.vector.y = 0.2;
    // direction.vector.z = 0;
    // dualArmRobot.linearMove(direction, true, true,true);
    // ROS_INFO("========== PUT BOX DOWN =================");
    // direction.vector.x = 0;
    // direction.vector.y = 0;
    // direction.vector.z = -0.1;
    // dualArmRobot.linearMove(direction, true, true,true);
    // sleep(1);
    // ROS_INFO("========== MOVE AWAY =================");
    // dualArmRobot.graspMove(-0.02);
    // sleep(1);
    // ROS_INFO("========== MOVE HOME POSITION =================");
    // dualArmRobot.moveHome();
    // sleep(1);
    ur_logger.stop();

   


    // END
    ros::shutdown();
    return 0;
}