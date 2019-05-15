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
    // MoveIt! motion planning variables
    moveit::planning_interface::MoveGroupInterface::Plan left_plan;
    moveit::planning_interface::MoveGroupInterface::Plan right_plan;
    moveit::planning_interface::MoveItErrorCode error;
    error.val = -1;
    // Start listening the FT sensor
    FTSensorSubscriber left_wrench_sub(nh, "left");

    // Set some joint constraints
    //@TODO verify its effectiveness
    dualArmRobot.setConstraints();
    dualArmRobot.kinematic_statePtr->enforceBounds();

    ROS_INFO("========== MOVE HOME POSITION =================");
    dualArmRobot.moveHome();

   // box7 goal pose
    geometry_msgs::PoseStamped box7_goal_pose_stamped;
    box7_goal_pose_stamped.pose.position.x = -0.671;
    box7_goal_pose_stamped.pose.position.x = -0.671;
    box7_goal_pose_stamped.pose.position.x = -0.671;

    box7_goal_pose_stamped.header = dualArmRobot.left_current_pose_.header;

    dualArmRobot.left_current_pose_ = dualArmRobot.left_.getCurrentPose(dualArmRobot.left_.getEndEffectorLink());
    box7_goal_pose_stamped.pose = dualArmRobot.left_current_pose_.pose;
    KDL::Frame left_frame_eef; // endeffector frame
    dual_arm_toolbox::Transform::transformPoseToKDL(dualArmRobot.left_current_pose_.pose, left_frame_eef);
    ROS_INFO_STREAM("Left position in world frame\n" << left_frame_eef.p);
    KDL::Rotation left_rot = left_frame_eef.M;
    // KDL::Rotation rotationAgnle = KDL::Rotation::Identity();
    double yaw = 0;   // Z-axis
    double pitch = 0; // Y-axis
    double roll = 0;  // X-axis
    double angle = 0;
    left_rot.GetEulerZYX(yaw, pitch, roll);

    ROS_INFO("Before roll = %f\tpitch = %f\t yaw = %f", roll, pitch, yaw);
    angle = 3.14/12;
    roll += angle;
    KDL::Rotation rot_in_world = KDL::Rotation::Identity();
    rot_in_world.DoRotX(angle); // Apply a rotation to *this (in its own frame)
    left_rot = left_rot.Inverse()*rot_in_world; 
    // = KDL::Rotation::RPY(roll, pitch, yaw);

    left_rot.GetEulerZYX(yaw, pitch, roll);
    left_rot.GetQuaternion(box7_goal_pose_stamped.pose.orientation.x,
                           box7_goal_pose_stamped.pose.orientation.y,
                           box7_goal_pose_stamped.pose.orientation.z,
                           box7_goal_pose_stamped.pose.orientation.w);
    ROS_INFO("After roll = %f\tpitch = %f\t yaw = %f", roll, pitch, yaw);
    dualArmRobot.moveObject("box7", box7_goal_pose_stamped, 0.1);
    sleep(5);

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
    ROS_INFO("Finished demonstration");
    ros::shutdown();
    return 0;
}