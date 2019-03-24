//
// Created by Chunting  on 08.04.17.
// rosrun dual_arm_robot_applications dual_arm_robot_lift_experiment_planning


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

// UR Logger
#include "ur_logging/UrLogger.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "dual_arm_robot_demonstration");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle nh;

    // Dual Arm Robot Setup
    dual_arm_demonstrator_iml::DualArmRobot dualArmRobot(nh);

    // Scene Setup
    dual_arm_demonstrator_iml::SceneManager sceneManager(nh);
    sceneManager.setupScene();
    // Start logging data
    std::vector<std::string> ur_namespaces;
    ur_namespaces.push_back("left");
    ur_namespaces.push_back("right");
    UR_Logger ur_logger(nh, ur_namespaces);
    ur_logger.start(100);
    // MoveIt! motion planning variables
    moveit::planning_interface::MoveGroupInterface::Plan left_plan;
    moveit::planning_interface::MoveGroupInterface::Plan right_plan;
    moveit::planning_interface::MoveItErrorCode error;
    error.val = -1;
    // Start listening the FT sensor
    FTSensorSubscriber FTsubscriber(nh, ur_namespaces[0]);

    // Set some joint constraints
    //@TODO verify its effectiveness
    dualArmRobot.setConstraints();
    dualArmRobot.kinematic_state->enforceBounds();

    // ROS_INFO("========== MOVE HOME POSITION =================");
    // dualArmRobot.moveHome();

    ROS_INFO("==========  Linear move up parallely  =================");
    geometry_msgs::Vector3Stamped direction;
    direction.header.frame_id = "world";
    direction.vector.x = 0;
    direction.vector.y = 0;
    direction.vector.z = 0.05;
    dualArmRobot.linearMoveParallel(direction, "box7", 0.1);

    // Pick box1
    // geometry_msgs::Vector3Stamped direction;
    // direction.header.frame_id = "/world";
    // direction.vector.x = 0;
    // direction.vector.y = 0;
    // direction.vector.z = 0.1;
    // if (!dualArmRobot.pickBox("box7", direction)) {
    //     ROS_WARN("Pick failed");
    //     ROS_ERROR("Can't execute demonstration without successful pick. Demonstration aborted.");
    //     return 0;
    // }

    // // clear path constraints
    // dualArmRobot.left_.clearPathConstraints();
    // dualArmRobot.right_.clearPathConstraints();

    // // plan move up
    // geometry_msgs::PoseStamped start_pose = dualArmRobot.left_.getCurrentPose(dualArmRobot.left_.getEndEffectorLink());
    // geometry_msgs::PoseStamped lift_pose = start_pose;
    // lift_pose.pose.position.z = lift_pose.pose.position.z + 0.6;

    // for (unsigned int i =0; i < 15 ; i++){
    //     ROS_INFO(":::::: START EVALUATION %i::::::", i);
    //     dualArmRobot.planMoveObject("box7", lift_pose, 0.2);
    //     ROS_INFO(":::::: END EVALUATION %i::::::", i);
    // }


    // END
    sleep(1);
    ur_logger.stop();
    ROS_INFO("Finished demonstration");
    ros::shutdown();
    return 0;
}