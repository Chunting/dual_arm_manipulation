//
// Created by Chunting  on 08.04.17.
//

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

// Robotiq 2 finger gripper
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "dual_arm_robot_grasp_test");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle nh;

    #ifdef OFFLINE
    ROS_WARN("Robot offline");
    #endif

    // Dual Arm Robot Setup
    dual_arm_demonstrator_iml::DualArmRobot dualArmRobot(nh);

    // Scene Setup
    dual_arm_demonstrator_iml::SceneManager sceneManager(nh);
    sceneManager.setupSceneLift();
    
    // Data log set-up
    std::vector<std::string> ur_namespaces;
    ur_namespaces.push_back("left");
    ur_namespaces.push_back("right");
    UR_Logger ur_logger(nh, ur_namespaces);
    ur_logger.start(100);

    // plan variables
    moveit::planning_interface::MoveGroup::Plan left_plan;
    moveit::planning_interface::MoveGroup::Plan right_plan;
    moveit::planning_interface::MoveItErrorCode error;
    error.val = -1;

    geometry_msgs::Vector3Stamped direction;
    direction.header.frame_id = "world";

    // move home
    dualArmRobot.moveHome();

    ros::Publisher pub_to_gripper;

    pub_to_gripper = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output> ("Robotiq2FGripperRobotOutput", 1);

    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output Grip;

    Grip.rACT = 0;
    Grip.rPR = 0;
    Grip.rGTO = 0;
    Grip.rSP  = 0;
    Grip.rFR = 0;
    Grip.rATR =0;

    pub_to_gripper.publish(Grip);
    sleep(2);

    Grip.rACT = 1;
    Grip.rPR = 0;
    Grip.rGTO = 1;
    Grip.rSP  = 255;
    Grip.rFR = 150;
    Grip.rATR =0;

    pub_to_gripper.publish(Grip);

    sleep(2);

    for(int i=0;i<10;i++){
        Grip.rPR = 255;
        pub_to_gripper.publish(Grip);
        sleep(2);
        Grip.rPR = 0;
        pub_to_gripper.publish(Grip);
        sleep(2);
    }

    sleep(1);
    ur_logger.stop();
    // END
    ROS_INFO("Finished demonstration");
    ros::shutdown();
    return 0;
}