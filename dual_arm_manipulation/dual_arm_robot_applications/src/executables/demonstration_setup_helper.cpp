// rosrun dual_arm_robot_applications demonstration_setup_helper
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

int main(int argc, char **argv)
{
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
    moveit::planning_interface::MoveGroupInterface::Plan left_plan;
    moveit::planning_interface::MoveGroupInterface::Plan right_plan;
    moveit::planning_interface::MoveItErrorCode error;
    error.val = -1;
    
    FTSensorSubscriber FTsubscriber(nh, "left");

    geometry_msgs::Vector3Stamped direction;
    direction.header.frame_id = "world";
    dualArmRobot.setConstraints();
    dualArmRobot.kinematic_state->enforceBounds();
    ROS_INFO("========== MOVE HOME POSITION =================");
    dualArmRobot.moveHome();
    sleep(1);

    // ROS_INFO("========== TEST IK  -- LEFT =================");
    // geometry_msgs::PoseStamped left_pose = dualArmRobot.left_.getCurrentPose(dualArmRobot.left_.getEndEffectorLink());
    // dualArmRobot.PrintPose(left_pose.pose);

    // std::vector<double> left_joint_values = dualArmRobot.getPositionIK(dualArmRobot.left_joint_model_group, left_pose.pose);

    // // ROS_INFO_STREAM("Current state is " <<dualArmRobot.kinematic_state->satisfiesBounds(left_joint_model_group));

    // // fk -> pos left
    // KDL::Frame frame_pose_left;
    // dualArmRobot.kinematic_state->setJointGroupPositions(dualArmRobot.left_joint_model_group, left_joint_values);
    // const Eigen::Affine3d &end_effector_pose_left = dualArmRobot.kinematic_state->getGlobalLinkTransform(dualArmRobot.left_.getEndEffectorLink());
    // tf::transformEigenToKDL(end_effector_pose_left, frame_pose_left);

    // geometry_msgs::Pose left_temp_pose;
    // dual_arm_toolbox::Transform::transformKDLtoPose(frame_pose_left, left_temp_pose);
    // ROS_INFO("FK left end_effector: %s", dualArmRobot.left_.getEndEffectorLink().c_str());
    // dualArmRobot.PrintPose(left_temp_pose);

    //     ROS_INFO("========== TEST IK  -- RIGHT =================");
    //     geometry_msgs::PoseStamped right_pose = dualArmRobot.right_.getCurrentPose(dualArmRobot.right_.getEndEffectorLink());
    //     ROS_INFO("Current Pose: ");
    //     dualArmRobot.PrintPose(right_pose.pose);
    //     moveit_msgs::RobotState seed_robot_state = dualArmRobot.getCurrentRobotStateMsg();
    //     std::string groupName = dualArmRobot.right_.getName();
    //     moveit_msgs::RobotState IK_robot_state = dualArmRobot.getPositionIK(groupName, seed_robot_state, right_pose);

    //     // fk -> pos right
    //    std::string right_endeffector = dualArmRobot.right_.getEndEffectorLink();
    //    geometry_msgs::PoseStamped FK_PoseStamped = dualArmRobot.publishPlanCartTrajectory(right_endeffector, IK_robot_state);

    //     std::vector<double> right_joint_values = dualArmRobot.right_.getCurrentJointValues();
    //     KDL::Frame frame_pose_right;
    //     dualArmRobot.kinematic_state->setJointGroupPositions(dualArmRobot.right_joint_model_group, right_joint_values);
    //     const Eigen::Affine3d &end_effector_pose_right = dualArmRobot.kinematic_state->getGlobalLinkTransform(dualArmRobot.right_.getEndEffectorLink());
    //     tf::transformEigenToKDL(end_effector_pose_right, frame_pose_right);

    //     geometry_msgs::Pose right_temp_pose;
    //     dual_arm_toolbox::Transform::transformKDLtoPose(frame_pose_right, right_temp_pose);
    //     ROS_INFO("frame_pose_right end_effector: %s", dualArmRobot.right_.getEndEffectorLink().c_str());
    //     dualArmRobot.PrintPose(FK_PoseStamped.pose);

    ROS_INFO("========== MOVE GRASP POSITION =================");
    dualArmRobot.moveGraspPosition();
    sleep(1);

    ROS_INFO("========== MOVE CLOSER =================");

    dualArmRobot.graspMove(0.025, false);
    double res_force = sqrt(FTsubscriber.last_wrench_msg_.wrench.force.x * FTsubscriber.last_wrench_msg_.wrench.force.x + FTsubscriber.last_wrench_msg_.wrench.force.y * FTsubscriber.last_wrench_msg_.wrench.force.y + FTsubscriber.last_wrench_msg_.wrench.force.z * FTsubscriber.last_wrench_msg_.wrench.force.z);

    while (res_force < 20)
    {
        dualArmRobot.graspMove(0.001);
        res_force = sqrt(FTsubscriber.last_wrench_msg_.wrench.force.x * FTsubscriber.last_wrench_msg_.wrench.force.x + FTsubscriber.last_wrench_msg_.wrench.force.y * FTsubscriber.last_wrench_msg_.wrench.force.y + FTsubscriber.last_wrench_msg_.wrench.force.z * FTsubscriber.last_wrench_msg_.wrench.force.z);
    }
    ROS_INFO("I heard: Force [%f]  FX[%f] FY[%f] FZ[%f]", res_force, FTsubscriber.last_wrench_msg_.wrench.force.x, FTsubscriber.last_wrench_msg_.wrench.force.y, FTsubscriber.last_wrench_msg_.wrench.force.z);

    sleep(1);
    // ROS_INFO("========== GET OFFSET =================");
    // dualArmRobot.getCurrentOffset();
    // sleep(1);
    ROS_INFO("========== LIFT BOX UP =================");
    // ROS_INFO("========== TEST LINEARMOVEPARALLEL =================");
    // direction.vector.x = 0;
    // direction.vector.y = 0;
    // direction.vector.z = 0.04;
    // dualArmRobot.linearMoveParallel(direction,"box7", 0.5);
    direction.vector.x = 0;
    direction.vector.y = 0;
    direction.vector.z = 0.05;
    dualArmRobot.linearMove(direction, true, true, true);
    // int stepnum = 0;
    // while(stepnum<6){
    //     if(res_force >= 60 && res_force <= 70){
    //         dualArmRobot.linearMove(direction, true, true,true);
    //     } else if(res_force > 70){
    //         // direction.vector.y = -0.001;
    //         dualArmRobot.graspMove(-0.001);
    //         dualArmRobot.linearMove(direction, true, true,true);
    //     }
    //     dualArmRobot.linearMove(direction, true, true,true);
    //     res_force = sqrt(FTsubscriber.last_wrench_msg_.wrench.force.x*FTsubscriber.last_wrench_msg_.wrench.force.x
    //                         + FTsubscriber.last_wrench_msg_.wrench.force.y*FTsubscriber.last_wrench_msg_.wrench.force.y
    //                         + FTsubscriber.last_wrench_msg_.wrench.force.z*FTsubscriber.last_wrench_msg_.wrench.force.z);
    // }
    // while (res_force >= 60 && res_force < 90){
    //     dualArmRobot.linearMove(direction, true, true,true);
    //     res_force = sqrt(FTsubscriber.last_wrench_msg_.wrench.force.x*FTsubscriber.last_wrench_msg_.wrench.force.x
    //                         + FTsubscriber.last_wrench_msg_.wrench.force.y*FTsubscriber.last_wrench_msg_.wrench.force.y
    //                         + FTsubscriber.last_wrench_msg_.wrench.force.z*FTsubscriber.last_wrench_msg_.wrench.force.z);
    // }
    // dualArmRobot.linearMoveParallel(direction,"box7", 0.5);

    sleep(1);

    ROS_INFO("========== MOVE LEFT =================");
    direction.vector.x = 0;
    direction.vector.y = -0.05;
    direction.vector.z = 0;
    dualArmRobot.linearMove(direction, true, true, true);
    // sleep(1);
    ROS_INFO("========== MOVE FORWARD =================");
    direction.vector.x = -0.05;
    direction.vector.y = 0;
    direction.vector.z = 0;
    dualArmRobot.linearMove(direction, true, true, true);
    // sleep(1);
    ROS_INFO("========== MOVE RIGHT =================");
    direction.vector.x = 0;
    direction.vector.y = 0.05;
    direction.vector.z = 0;
    dualArmRobot.linearMove(direction, true, true, true);
    // sleep(1);
    // ROS_INFO("========== MOVE BACK =================");
    // direction.vector.x = 0.05;
    // direction.vector.y = 0;
    // direction.vector.z = 0;
    // dualArmRobot.linearMove(direction, true, true,true);
    // sleep(1);

    ROS_INFO("========== PUT BOX DOWN =================");
    direction.vector.x = 0;
    direction.vector.y = 0;
    direction.vector.z = -0.05;
    // dualArmRobot.linearMoveParallel(direction,"box7", 0.5);
    dualArmRobot.linearMove(direction, true, true, true);
    sleep(1);
    ROS_INFO("========== MOVE AWAY =================");
    dualArmRobot.graspMove(-0.01, false);
    // sleep(1);
    ROS_INFO("========== MOVE HOME POSITION =================");
    dualArmRobot.moveHome();

    sleep(1);
    // END
    ros::shutdown();
    return 0;
}