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
    ros::Time before_pick_7;
    ros::Duration manipulation_7;
    ros::Time after_place_7;

    before_pick_7 = ros::Time::now();
    FTSensorSubscriber left_wrench_sub(nh, "left");
    FTSensorSubscriber right_wrench_sub(nh, "right");
    ROS_INFO("========== MOVE HOME POSITION =================");
    dualArmRobot.moveHome();
    ROS_INFO("========== MOVE GRASP POSITION =================");
    dualArmRobot.moveGraspPosition();
    ROS_INFO("========== MOVE CLOSER =================");
    dualArmRobot.graspMove(0.055, false, true, true);
    double res_force = sqrt(left_wrench_sub.last_wrench_msg_.wrench.force.x * left_wrench_sub.last_wrench_msg_.wrench.force.x 
                            + left_wrench_sub.last_wrench_msg_.wrench.force.y * left_wrench_sub.last_wrench_msg_.wrench.force.y 
                            + left_wrench_sub.last_wrench_msg_.wrench.force.z * left_wrench_sub.last_wrench_msg_.wrench.force.z);

    while (res_force < 20)
    {
        dualArmRobot.graspMove(0.001, false, true, true, 0.1); // true : left arm; false: right arm
        sleep(1);
        res_force = sqrt(left_wrench_sub.last_wrench_msg_.wrench.force.x * left_wrench_sub.last_wrench_msg_.wrench.force.x 
                    + left_wrench_sub.last_wrench_msg_.wrench.force.y * left_wrench_sub.last_wrench_msg_.wrench.force.y 
                    + left_wrench_sub.last_wrench_msg_.wrench.force.z * left_wrench_sub.last_wrench_msg_.wrench.force.z);
        ROS_INFO("I heard: Force [%f]  FX[%f] FY[%f] FZ[%f]", res_force, left_wrench_sub.last_wrench_msg_.wrench.force.x, left_wrench_sub.last_wrench_msg_.wrench.force.y, left_wrench_sub.last_wrench_msg_.wrench.force.z);
    }
    ROS_INFO("Desired force: Force [%f]  FX[%f] FY[%f] FZ[%f]", res_force, left_wrench_sub.last_wrench_msg_.wrench.force.x, left_wrench_sub.last_wrench_msg_.wrench.force.y, left_wrench_sub.last_wrench_msg_.wrench.force.z);
    KDL::Frame desired_offset = dualArmRobot.getCurrentOffset(); 
    Vector3d offset_position;
    Eigen::Quaterniond offset_quaternion;
    tf::vectorKDLToEigen (desired_offset.p, offset_position);
    tf::quaternionKDLToEigen (desired_offset.M, offset_quaternion);
    Vector6d offset_vec;
    offset_vec.topRows(3) = offset_position;
    offset_vec.bottomRows(3) =  offset_quaternion.toRotationMatrix().eulerAngles(0,1,2);
    std::cout << "Desired offset in left arm frame (in roll, pitch, yaw)\n" << offset_vec << std::endl;
    ROS_INFO("========== PICK UP =================");
    // Pick box7 on top
    geometry_msgs::Vector3Stamped direction;
    direction.header.frame_id = "world";
    direction.vector.x = 0.1;
    direction.vector.y = 0;
    direction.vector.z = 0.25;
    if (!dualArmRobot.pickBox("box7", direction))
    {
        ROS_WARN("Pick failed");
        ROS_ERROR("Can't execute demonstration without successful pick. Demonstration aborted.");
        return 0;
    }
    ROS_INFO("========== Rotation =================");
    // box7 goal pose
    geometry_msgs::PoseStamped box7_goal_pose_stamped;
    geometry_msgs::Pose left_waypoint_pose, right_waypoint_pose;
    std::vector< geometry_msgs::Pose > left_waypoints_pose_vec;
    dualArmRobot.left_.setStartStateToCurrentState();
    dualArmRobot.right_.setStartStateToCurrentState();
    dualArmRobot.left_current_pose_ = dualArmRobot.left_.getCurrentPose(dualArmRobot.left_.getEndEffectorLink());
    dualArmRobot.right_current_pose_ = dualArmRobot.right_.getCurrentPose(dualArmRobot.right_.getEndEffectorLink());
    left_waypoint_pose = dualArmRobot.left_current_pose_.pose;
    right_waypoint_pose = dualArmRobot.right_current_pose_.pose;
    double x_center = (left_waypoint_pose.position.x + right_waypoint_pose.position.x)/2;
    double y_center = (left_waypoint_pose.position.y + right_waypoint_pose.position.y)/2;
    double z_center = (left_waypoint_pose.position.z + right_waypoint_pose.position.z)/2;
    double radius = std::abs((left_waypoint_pose.position.y - right_waypoint_pose.position.y)/2);
    double d_angle = 10*3.14159/180; // 3 degree
    double angle = 0;
    // ROS_INFO("y_center = %f,  left_y = %f,  right_y = %f", y_center, left_waypoint_pose.position.y, right_waypoint_pose.position.y);
    // ROS_INFO("z_center = %f,  left_z = %f,  right_z = %f", z_center, left_waypoint_pose.position.z, right_waypoint_pose.position.z);
    // ROS_INFO("x_center = %f,  left_x = %f,  right_x = %f", x_center, left_waypoint_pose.position.x, right_waypoint_pose.position.x);
    // ROS_INFO("radius = %f", radius);
    left_waypoints_pose_vec.push_back(left_waypoint_pose);
    
    
    // box7_goal_pose_stamped.header = dualArmRobot.left_current_pose_.header;
    // box7_goal_pose_stamped.pose = dualArmRobot.left_current_pose_.pose;
    KDL::Frame left_frame_eef; // endeffector frame
    dual_arm_toolbox::Transform::transformPoseToKDL(left_waypoint_pose, left_frame_eef);
    KDL::Rotation left_rot = left_frame_eef.M;
    double yaw = 0, pitch = 0, roll = 0;   // Z-axis
    
    int num_points = 5;
    double step_size = angle/num_points;
    left_rot.GetEulerZYX(yaw, pitch, roll);
    // ROS_INFO("1............Print Orientation: roll=%f,  pitch=%f,  yaw=%f\n", roll, pitch, yaw);

    // roll += angle;
    // left_rot = KDL::Rotation::EulerZYX(yaw, pitch, roll);
    // left_rot.GetEulerZYX(yaw, pitch, roll);
    // left_rot.GetQuaternion(box7_goal_pose_stamped.pose.orientation.x,
    //                        box7_goal_pose_stamped.pose.orientation.y,
    //                        box7_goal_pose_stamped.pose.orientation.z,
    //                        box7_goal_pose_stamped.pose.orientation.w);
    
    for(int i=0; i<num_points; ++i){
        angle += d_angle;
        left_waypoint_pose.position.x = x_center;
        left_waypoint_pose.position.y = y_center - radius*cos(angle);
        left_waypoint_pose.position.z = z_center + radius*sin(angle);
        
        roll -= d_angle;
        left_rot = KDL::Rotation::EulerZYX(yaw, pitch, roll);
        left_rot.GetQuaternion(left_waypoint_pose.orientation.x,
                           left_waypoint_pose.orientation.y,
                           left_waypoint_pose.orientation.z,
                           left_waypoint_pose.orientation.w);
        left_waypoints_pose_vec.push_back(left_waypoint_pose);
        // ROS_INFO("x=%f, y=%f, z=%f, roll=%f, angle=%f, d_angle=%f", 
        //     left_waypoint_pose.position.x, left_waypoint_pose.position.y, left_waypoint_pose.position.z,
        //     roll, angle, d_angle);

    }
    dualArmRobot.moveObject("box7", left_waypoints_pose_vec, 0.25);
    left_waypoints_pose_vec.clear();
    left_waypoints_pose_vec.push_back(left_waypoint_pose);

   
    // ROS_INFO("2.....x=%f, y=%f, z=%f, roll=%f, angle=%f, d_angle=%f", 
    //         left_waypoint_pose.position.x, left_waypoint_pose.position.y, left_waypoint_pose.position.z,
    //         roll, angle, d_angle);

    for(int i=0; i<num_points; ++i){
        angle -= d_angle;
        left_waypoint_pose.position.x = x_center;
        left_waypoint_pose.position.y = y_center - radius*cos(angle);
        left_waypoint_pose.position.z = z_center + radius*sin(angle);

        roll += d_angle;
        left_rot = KDL::Rotation::EulerZYX(yaw, pitch, roll);
        left_rot.GetQuaternion(left_waypoint_pose.orientation.x,
                           left_waypoint_pose.orientation.y,
                           left_waypoint_pose.orientation.z,
                           left_waypoint_pose.orientation.w);
        left_waypoints_pose_vec.push_back(left_waypoint_pose);
        // ROS_INFO("x=%f, y=%f, z=%f, roll=%f, angle=%f, d_angle=%f", 
        //     left_waypoint_pose.position.x, left_waypoint_pose.position.y, left_waypoint_pose.position.z,
        //     roll, angle, d_angle);
    }
    // ROS_INFO("3.....x=%f, y=%f, z=%f, roll=%f, angle=%f, d_angle=%f", 
    //         left_waypoint_pose.position.x, left_waypoint_pose.position.y, left_waypoint_pose.position.z,
    //         roll, angle, d_angle);

    // roll -= angle;
    // left_rot = KDL::Rotation::RPY(roll, pitch, yaw);
    // left_rot.GetEulerZYX(yaw, pitch, roll);
    // left_rot.GetQuaternion(box7_goal_pose_stamped.pose.orientation.x,
    //                        box7_goal_pose_stamped.pose.orientation.y,
    //                        box7_goal_pose_stamped.pose.orientation.z,
    //                        box7_goal_pose_stamped.pose.orientation.w);
    dualArmRobot.moveObject("box7", left_waypoints_pose_vec, 0.25);
    left_waypoints_pose_vec.clear();

    ROS_INFO("========== PLACE DOWN =================");
    // Place box7
    direction.vector.x = -direction.vector.x;
    direction.vector.y = -direction.vector.y;
    direction.vector.z = -direction.vector.z;
    if (!dualArmRobot.placeBox("box7", direction))
    {
        ROS_WARN("Place Box failed");
        ROS_ERROR("Demonstration aborted to avoid further problems");
        return 0;
    }

    dualArmRobot.moveHome();
    after_place_7 = ros::Time::now();
    manipulation_7 = after_place_7 - before_pick_7;
    ROS_INFO(":::::: VALUES EVALUATION ::::::");
    ROS_INFO("manipulation box 7 took: %li sec", manipulation_7.toSec());
    ROS_INFO("Finished demonstration");
    ros::shutdown();
    return 0;
}