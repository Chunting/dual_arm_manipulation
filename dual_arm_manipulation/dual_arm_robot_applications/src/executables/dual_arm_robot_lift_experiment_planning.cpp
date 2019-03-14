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


int main(int argc, char **argv) {
    ros::init(argc, argv, "dual_arm_robot_demonstration");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle nh;

    #ifdef OFFLINE
    ROS_WARN("Robot offline");
    #endif

    /* Dual Arm Robot Setup */
    dual_arm_demonstrator_iml::DualArmRobot dualArmRobot(nh);

    /* Scene Setup */
    dual_arm_demonstrator_iml::SceneManager sceneManager(nh);
    
    // Planning to a joint-space goal
    std::vector<double> group_joint_values;
    // read the current joint angles
    group_joint_values = dualArmRobot.getJointAngles("arms");
    //group_joint_values.clear();

    // with or without collision objects?
    // sceneManager.setupSceneLiftCO();
    //  sceneManager.setupSceneLift();

    // move home
    // sleep(5);
    dualArmRobot.moveHome();
    sleep(5);
    group_joint_values = dualArmRobot.getJointAngles("arms");


    // setup constraints
    moveit_msgs::JointConstraint jcm;
    moveit_msgs::Constraints left_constraints;
    moveit_msgs::Constraints right_constraints;
    moveit_msgs::Constraints both_constraints;
    ROS_INFO("Start to set up the constraints...");

    jcm.joint_name="left_shoulder_pan_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 0.5;
    jcm.tolerance_below = 2;
    jcm.weight = 1.0;
    left_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.left_.setPathConstraints(left_constraints);

    // ur5 can get blocked while placing without this constraint
    jcm.joint_name="left_elbow_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 3;
    jcm.tolerance_below = 0.1;
    jcm.weight = 0.5;
    left_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.left_.setPathConstraints(left_constraints);

    
    // when placing box on top ur5 can get blocked because wrist 1 reaches limit
    jcm.joint_name="left_wrist_3_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 3.0;
    jcm.tolerance_below = 3.0;
    jcm.weight = 1;
    left_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.left_.setPathConstraints(left_constraints);

    jcm.joint_name="right_shoulder_pan_joint";
    jcm.position = 0.01;
    jcm.tolerance_above = 3;
    jcm.tolerance_below = 3;
    jcm.weight = 1.0;
    left_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.left_.setPathConstraints(left_constraints);

    jcm.joint_name="right_wrist_2_joint";
    jcm.position = 0;
    jcm.tolerance_above = 3;
    jcm.tolerance_below = 3;
    jcm.weight = 1.0;
    right_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.right_.setPathConstraints(right_constraints);

    dualArmRobot.arms_.setPathConstraints(both_constraints);
    ROS_INFO("Finished setting up the constraints...");

    /*
    moveit_msgs::JointConstraint jcm;
    moveit_msgs::Constraints left_constraints;
    moveit_msgs::Constraints right_constraints;
    jcm.joint_name="right_shoulder_pan_joint";
    jcm.position = -2.4;
    jcm.tolerance_above = 2.5;
    jcm.tolerance_below = 0.7;
    jcm.weight = 1.0;
    right_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.right_.setPathConstraints(right_constraints);*/
    /*
    jcm.joint_name="right_wrist_2_joint";
    jcm.position = 3.0;
    jcm.tolerance_above = 1.0;
    jcm.tolerance_below = 4.0;
    jcm.weight = 1.0;
    right_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.right_.setPathConstraints(right_constraints);*/
/*
    jcm.joint_name="left_shoulder_pan_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 1.0;
    jcm.tolerance_below = 1.0;
    jcm.weight = 1.0;
    left_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.left_.setPathConstraints(left_constraints);*/
/*
    jcm.joint_name="left_shoulder_pan_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 0.4;
    jcm.tolerance_below = 0.4;
    jcm.weight = 1.0;
    left_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.left_.setPathConstraints(left_constraints);
*/
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
    ROS_INFO("Finished demonstration");
    ros::shutdown();
    return 0;
}