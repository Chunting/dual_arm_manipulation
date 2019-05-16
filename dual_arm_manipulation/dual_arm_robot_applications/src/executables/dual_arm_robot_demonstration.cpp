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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_arm_robot_demonstration");
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
    sceneManager.setupScene();

    FTSensorSubscriber left_wrench_sub(nh, "left");
    FTSensorSubscriber right_wrench_sub(nh, "right");
    ROS_INFO("========== MOVE HOME POSITION =================");
    dualArmRobot.moveHome();
    sleep(1);
    ROS_INFO("========== MOVE GRASP POSITION =================");
    dualArmRobot.moveGraspPosition();
    ROS_INFO("========== MOVE CLOSER =================");
    dualArmRobot.graspMove(0.017, false, true, true);
    double res_force = sqrt(left_wrench_sub.last_wrench_msg_.wrench.force.x * left_wrench_sub.last_wrench_msg_.wrench.force.x + left_wrench_sub.last_wrench_msg_.wrench.force.y * left_wrench_sub.last_wrench_msg_.wrench.force.y + left_wrench_sub.last_wrench_msg_.wrench.force.z * left_wrench_sub.last_wrench_msg_.wrench.force.z);

    while (res_force < 20)
    {
        dualArmRobot.graspMove(0.001, false, true, false); // true : left arm; false: right arm
        res_force = sqrt(left_wrench_sub.last_wrench_msg_.wrench.force.x * left_wrench_sub.last_wrench_msg_.wrench.force.x + left_wrench_sub.last_wrench_msg_.wrench.force.y * left_wrench_sub.last_wrench_msg_.wrench.force.y + left_wrench_sub.last_wrench_msg_.wrench.force.z * left_wrench_sub.last_wrench_msg_.wrench.force.z);
        ROS_INFO("I heard: Force [%f]  FX[%f] FY[%f] FZ[%f]", res_force, left_wrench_sub.last_wrench_msg_.wrench.force.x, left_wrench_sub.last_wrench_msg_.wrench.force.y, left_wrench_sub.last_wrench_msg_.wrench.force.z);
        
    }
    KDL::Frame desired_offset = dualArmRobot.getCurrentOffset(); 
    Vector3d offset_position;
    Eigen::Quaterniond offset_quaternion;
    tf::vectorKDLToEigen (desired_offset.p, offset_position);
    tf::quaternionKDLToEigen (desired_offset.M, offset_quaternion);
    Vector6d offset_vec;
    offset_vec.topRows(3) = offset_position;
    offset_vec.bottomRows(3) =  offset_quaternion.toRotationMatrix().eulerAngles(0,1,2);
    std::cout << "Desired offset in left arm frame (in roll, pitch, yaw)\n" << offset_vec << std::endl;
    sleep(5);
    ROS_INFO("========== PICK UP =================");
    // Eval
    ros::Time before_pick_7;
    ros::Duration manipulation_7;
    ros::Time after_place_7;

    before_pick_7 = ros::Time::now();
    // Pick box7 on top
    geometry_msgs::Vector3Stamped direction;
    direction.header.frame_id = "world";
    direction.vector.x = 0;
    direction.vector.y = 0;
    direction.vector.z = 0.20;
    if (!dualArmRobot.pickBox("box7", direction))
    {
        ROS_WARN("Pick failed");
        ROS_ERROR("Can't execute demonstration without successful pick. Demonstration aborted.");
        return 0;
    }
    // box7 goal pose
    geometry_msgs::PoseStamped box7_goal_pose_stamped;
    dualArmRobot.left_current_pose_ = dualArmRobot.left_.getCurrentPose(dualArmRobot.left_.getEndEffectorLink());
    box7_goal_pose_stamped.header = dualArmRobot.left_current_pose_.header;
    box7_goal_pose_stamped.pose = dualArmRobot.left_current_pose_.pose;
    KDL::Frame left_frame_eef; // endeffector frame
    dual_arm_toolbox::Transform::transformPoseToKDL(dualArmRobot.left_current_pose_.pose, left_frame_eef);
    ROS_INFO_STREAM("Left position in frame " << box7_goal_pose_stamped.header.frame_id << "\n" << left_frame_eef.p);
    KDL::Rotation left_rot = left_frame_eef.M;
    double yaw = 0;   // Z-axis
    double pitch = 0; // Y-axis
    double roll = 0;  // X-axis
    double angle = 0;
    left_rot.GetEulerZYX(yaw, pitch, roll);
    ROS_INFO("Before roll = %f\tpitch = %f\t yaw = %f", roll, pitch, yaw);
    angle = -3.14/6;
    roll += angle;
    left_rot = KDL::Rotation::EulerZYX(yaw, pitch, roll);
    left_rot.GetEulerZYX(yaw, pitch, roll);
    left_rot.GetQuaternion(box7_goal_pose_stamped.pose.orientation.x,
                           box7_goal_pose_stamped.pose.orientation.y,
                           box7_goal_pose_stamped.pose.orientation.z,
                           box7_goal_pose_stamped.pose.orientation.w);
    ROS_INFO("After roll = %f\tpitch = %f\t yaw = %f", roll, pitch, yaw);
    dualArmRobot.moveObject("box7", box7_goal_pose_stamped, 0.2);
    sleep(5);

    roll -= angle;
    left_rot = KDL::Rotation::RPY(roll, pitch, yaw);
    left_rot.GetEulerZYX(yaw, pitch, roll);
    ROS_INFO("After roll = %f\tpitch = %f\t yaw = %f", roll, pitch, yaw);
    left_rot.GetQuaternion(box7_goal_pose_stamped.pose.orientation.x,
                           box7_goal_pose_stamped.pose.orientation.y,
                           box7_goal_pose_stamped.pose.orientation.z,
                           box7_goal_pose_stamped.pose.orientation.w);
    dualArmRobot.moveObject("box7", box7_goal_pose_stamped, 0.2);
    sleep(5);
    ROS_INFO("========== PLACE DOWN =================");
    // Place box7
    geometry_msgs::Vector3 go_down;
    go_down.x = 0;
    go_down.y = 0;
    go_down.z = -direction.vector.z;
    if (!dualArmRobot.placeBox("box7", box7_goal_pose_stamped, go_down))
    {
        ROS_WARN("Place Box failed");
        ROS_ERROR("Demonstration aborted to avoid further problems");
        return 0;
    }
    after_place_7 = ros::Time::now();
    manipulation_7 = after_place_7 - before_pick_7;
    ROS_INFO(":::::: VALUES EVALUATION ::::::");
    ROS_INFO("manipulation box 7 took: %li nsec", manipulation_7.toNSec());
    sleep(1);
    dualArmRobot.moveHome();
    ROS_INFO("Finished demonstration");
    ros::shutdown();
    return 0;
}