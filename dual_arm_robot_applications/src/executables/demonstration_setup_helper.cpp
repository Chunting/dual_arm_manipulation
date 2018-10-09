//
// Created by Daniel Höltgen on 23.12.16.
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

    // move left to shelf
    while (error.val != 1){
        geometry_msgs::PoseStamped left_pose;
        left_pose.header.frame_id = "shelf";
        left_pose.pose.position.x = -0.07;
        left_pose.pose.position.y = 0.125;
        left_pose.pose.position.z = 0.53;
        KDL::Rotation left_rot;  // generated to easily assign quaternion of pose
        left_rot.DoRotY(3.14 / 2);
        left_rot.GetQuaternion(left_pose.pose.orientation.x, left_pose.pose.orientation.y, left_pose.pose.orientation.z,
                              left_pose.pose.orientation.w);
        dualArmRobot.left_.setPoseTarget(left_pose, dualArmRobot.left_.getEndEffectorLink());

        error = dualArmRobot.left_.plan(left_plan);

    }
    dualArmRobot.execute(left_plan);

    error.val = -1;
    while (error.val != 1){
        // move ur10 to shelf
        geometry_msgs::PoseStamped right_pose;
        right_pose.header.frame_id = "shelf";
        right_pose.pose.position.x = 0.125;
        right_pose.pose.position.y = 0.25 + 0.07;
        right_pose.pose.position.z = 0.53;
        KDL::Rotation right_rot;  // generated to easily assign quaternion of pose
        right_rot.DoRotX(3.14 / 2);
        right_rot.GetQuaternion(right_pose.pose.orientation.x, right_pose.pose.orientation.y, right_pose.pose.orientation.z,
                               right_pose.pose.orientation.w);
        dualArmRobot.right_.setPoseTarget(right_pose, dualArmRobot.right_.getEndEffectorLink());

        error = dualArmRobot.right_.plan(right_plan);

    }
    dualArmRobot.execute(right_plan);

    // move closer
    dualArmRobot.graspMove(0.07);

    // END
    ROS_INFO("Place the shelf between the robot arms as displayed in rviz!");
    ros::shutdown();
    return 0;
}