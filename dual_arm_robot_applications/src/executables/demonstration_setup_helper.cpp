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
   

 // setup constraints
    moveit_msgs::JointConstraint jcm;
    moveit_msgs::Constraints left_constraints;
    moveit_msgs::Constraints right_constraints;
    moveit_msgs::Constraints both_constraints;
    ROS_INFO("Start to set up the constraints...");
    // when placing box on top ur5 can get blocked because wrist 1 reaches limit
    jcm.joint_name="left_wrist_1_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 3.0;
    jcm.tolerance_below = 3.0;
    jcm.weight = 1;
    left_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);


    

    // ur5 sometimes blocks itself when picking the box on bottom, on top ur10 can get problems adapting its trajectory, this solve the issue.
    jcm.joint_name="left_shoulder_pan_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 3;
    jcm.tolerance_below = 3;
    jcm.weight = 1.0;
    left_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);


    jcm.joint_name="left_shoulder_lift_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 3;
    jcm.tolerance_below = 3;
    jcm.weight = 1.0;
    left_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);


    // ur5 can get blocked while placing without this constraint
    jcm.joint_name="left_elbow_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 3;
    jcm.tolerance_below = 3;
    jcm.weight = 0.5;
    left_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);


    // ur5 sometimes blocks itself when picking the box on bottom, on top ur10 can get problems adapting its trajectory, this solve the issue.
    jcm.joint_name="left_wrist_3_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 3;
    jcm.tolerance_below = 3;
    jcm.weight = 1.0;
    left_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    
    // ur5 sometimes blocks itself when picking the box on top, this should solve the issue

    jcm.joint_name="right_shoulder_pan_joint";
    jcm.position = 0.01;
    jcm.tolerance_above = 3;
    jcm.tolerance_below = 0;
    jcm.weight = 1.0;
    right_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);


    jcm.joint_name="right_shoulder_lift_joint";
    jcm.position = 0.01;
    jcm.tolerance_above = 3;
    jcm.tolerance_below = 3;
    jcm.weight = 1.0;
    right_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);


    jcm.joint_name="right_wrist_2_joint";
    jcm.position = 0;
    jcm.tolerance_above = 3;
    jcm.tolerance_below = 3;
    jcm.weight = 1.0;
    right_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);


    jcm.joint_name="right_wrist_3_joint";
    jcm.position = 0;
    jcm.tolerance_above = 3;
    jcm.tolerance_below = 3;
    jcm.weight = 1.0;
    right_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);

    dualArmRobot.left_.setPathConstraints(left_constraints);
    dualArmRobot.right_.setPathConstraints(right_constraints);
    dualArmRobot.arms_.setPathConstraints(both_constraints);
    ROS_INFO("Finished setting up the constraints...");


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

    dualArmRobot.kinematic_state->enforceBounds();

    ROS_INFO("========== TEST IK  -- LEFT =================");
    geometry_msgs::PoseStamped left_pose = dualArmRobot.left_.getCurrentPose(dualArmRobot.left_.getEndEffectorLink());
    dualArmRobot.PrintPose(left_pose.pose);
    std::vector<double> left_joint_values = dualArmRobot.getPositionIK(dualArmRobot.left_joint_model_group, left_pose.pose);

    // dualArmRobot.kinematic_state->setJointGroupPositions(dualArmRobot.left_joint_model_group, left_joint_values);
    // ROS_INFO_STREAM("Current state is " <<dualArmRobot.kinematic_state->satisfiesBounds(left_joint_model_group));

    // fk -> pos left
    KDL::Frame frame_pose_left;
    dualArmRobot.kinematic_state->setJointGroupPositions(dualArmRobot.left_joint_model_group, left_joint_values);
    const Eigen::Affine3d &end_effector_pose_left = dualArmRobot.kinematic_state->getGlobalLinkTransform(dualArmRobot.left_.getEndEffectorLink());
    tf::transformEigenToKDL(end_effector_pose_left, frame_pose_left);

    geometry_msgs::Pose left_temp_pose;
    dual_arm_toolbox::Transform::transformKDLtoPose(frame_pose_left, left_temp_pose);
    ROS_INFO("frame_pose_left end_effector: %s", dualArmRobot.left_.getEndEffectorLink().c_str());
    dualArmRobot.PrintPose(left_temp_pose);

    ROS_INFO("========== TEST IK  -- RIGHT =================");
    geometry_msgs::PoseStamped right_pose = dualArmRobot.right_.getCurrentPose(dualArmRobot.right_.getEndEffectorLink());
    dualArmRobot.PrintPose(right_pose.pose);
    moveit_msgs::RobotState seed_robot_state = dualArmRobot.getCurrentRobotStateMsg();
    std::string groupName = dualArmRobot.right_.getName();
    moveit_msgs::RobotState IK_robot_state = dualArmRobot.getPositionIK(groupName, seed_robot_state, right_pose);

    // fk -> pos left
   std::string right_endeffector = dualArmRobot.right_.getEndEffectorLink();
   geometry_msgs::PoseStamped FK_PoseStamped = dualArmRobot.getPositionFK(right_endeffector, IK_robot_state);



    // KDL::Frame frame_pose_right;
    // dualArmRobot.kinematic_state->setJointGroupPositions(dualArmRobot.right_joint_model_group, right_joint_values);
    // const Eigen::Affine3d &end_effector_pose_right = dualArmRobot.kinematic_state->getGlobalLinkTransform(dualArmRobot.right_.getEndEffectorLink());
    // tf::transformEigenToKDL(end_effector_pose_right, frame_pose_right);

    // geometry_msgs::Pose right_temp_pose;
    // dual_arm_toolbox::Transform::transformKDLtoPose(frame_pose_right, right_temp_pose);
    ROS_INFO("frame_pose_left end_effector: %s", dualArmRobot.right_.getEndEffectorLink().c_str());
    dualArmRobot.PrintPose(FK_PoseStamped.pose);
    // ROS_INFO("========== MOVE GRASP POSITION =================");
    // dualArmRobot.moveGraspPosition();
    // sleep(1);
    // ROS_INFO("========== MOVE CLOSER =================");
    // dualArmRobot.graspMove(0.01);
    // sleep(1);
    // ROS_INFO("========== GET OFFSET =================");
    // dualArmRobot.getCurrentOffset();
    // sleep(1);
    // ROS_INFO("========== LIFT BOX UP =================");
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
    ROS_INFO("========== PUT BOX DOWN =================");
    direction.vector.x = 0;
    direction.vector.y = -0.1;
    direction.vector.z = 0;
    dualArmRobot.linearMoveParallel(direction,"box7", 0.5);
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