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

// UR Logger
#include "ur_logging/UrLogger.h"

// #include "dual_arm_demonstrator_iml/FTSensorSubscriber.h"

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
    // Start logging data
    std::vector<std::string> ur_namespaces;
    ur_namespaces.push_back("left");
    ur_namespaces.push_back("right");
    UR_Logger ur_logger(nh, ur_namespaces);
    ur_logger.start(100);

    // variables
    moveit::planning_interface::MoveGroupInterface::Plan left_plan;
    moveit::planning_interface::MoveGroupInterface::Plan right_plan;
    moveit::planning_interface::MoveItErrorCode error;
    error.val = -1;

    FTSensorSubscriber FTsubscriber(nh, ur_namespaces[0]);

    geometry_msgs::Vector3Stamped direction;
    direction.header.frame_id = "world";
    dualArmRobot.setConstraints();
    dualArmRobot.kinematic_state->enforceBounds();
    
    ROS_INFO("========== MOVE HOME POSITION =================");
    dualArmRobot.moveHome();
    sleep(1);
    ROS_INFO("========== MOVE GRASP POSITION =================");
    dualArmRobot.moveGraspPosition();
    sleep(1);

    ROS_INFO("========== MOVE CLOSER =================");

    dualArmRobot.graspMove(0.06, false, true,false );
    double res_force = sqrt(FTsubscriber.last_wrench_msg_.wrench.force.x * FTsubscriber.last_wrench_msg_.wrench.force.x + FTsubscriber.last_wrench_msg_.wrench.force.y * FTsubscriber.last_wrench_msg_.wrench.force.y + FTsubscriber.last_wrench_msg_.wrench.force.z * FTsubscriber.last_wrench_msg_.wrench.force.z);

    while (res_force < 10)
    {
        dualArmRobot.graspMove(0.001, false, true, false); // true : left arm; false: right arm
        res_force = sqrt(FTsubscriber.last_wrench_msg_.wrench.force.x * FTsubscriber.last_wrench_msg_.wrench.force.x + FTsubscriber.last_wrench_msg_.wrench.force.y * FTsubscriber.last_wrench_msg_.wrench.force.y + FTsubscriber.last_wrench_msg_.wrench.force.z * FTsubscriber.last_wrench_msg_.wrench.force.z);
        ROS_INFO("I heard: Force [%f]  FX[%f] FY[%f] FZ[%f]", res_force, FTsubscriber.last_wrench_msg_.wrench.force.x, FTsubscriber.last_wrench_msg_.wrench.force.y, FTsubscriber.last_wrench_msg_.wrench.force.z);
    }
    
    ros::Publisher offset_desired_pub = nh.advertise<geometry_msgs::PointStamped>("/desired_offset_point", 1);
    // Publish the desired offset between two EEs, described in right EE coordinate system
    KDL::Frame desired_offset = dualArmRobot.getCurrentOffset().Inverse();  // w.r.t right_ee_link coordinate system
    geometry_msgs::PointStamped offset_point_temp_;
    offset_point_temp_.header.frame_id = dualArmRobot.right_.getEndEffectorLink();
    offset_point_temp_.point.x = desired_offset.p.x();
    offset_point_temp_.point.y = desired_offset.p.y();
    offset_point_temp_.point.z = desired_offset.p.z();
    offset_desired_pub.publish(offset_point_temp_);
    sleep(1);
    ROS_INFO("========== PICK UP =================");
    // Eval
    ros::Time before_pick_7;
    ros::Duration manipulation_7;
    ros::Time after_place_7;

    before_pick_7 = ros::Time::now();
    // Pick box7 on top
    direction.header.frame_id = "world";
    direction.vector.x = 0;
    direction.vector.y = 0;
    direction.vector.z = 0.15;
    if (!dualArmRobot.pickBox("box7", direction))
    {
        ROS_WARN("Pick failed");
        ROS_ERROR("Can't execute demonstration without successful pick. Demonstration aborted.");
        return 0;
    }

    // box7 goal pose
    geometry_msgs::PoseStamped box7_goal_pose_stamped;
    box7_goal_pose_stamped.header = dualArmRobot.left_current_pose_.header;

    dualArmRobot.left_current_pose_ = dualArmRobot.left_.getCurrentPose(dualArmRobot.left_.getEndEffectorLink());
    box7_goal_pose_stamped.pose = dualArmRobot.left_current_pose_.pose;
    KDL::Frame left_frame_eef; // endeffector frame
    dual_arm_toolbox::Transform::transformPoseToKDL(dualArmRobot.left_current_pose_.pose, left_frame_eef);
    KDL::Rotation left_rot = left_frame_eef.M;
    double alfa = 0;
    double beta = 0;
    double gamma = 0;
    left_rot.GetEulerZYX(alfa, beta, gamma);
    ROS_INFO("Before alfa = %f\tbeta = %f\t gamma = %f", alfa, beta, gamma);

    left_rot.DoRotX(-3.14 /12);
    left_rot.GetQuaternion(box7_goal_pose_stamped.pose.orientation.x, box7_goal_pose_stamped.pose.orientation.y, box7_goal_pose_stamped.pose.orientation.z,
                           box7_goal_pose_stamped.pose.orientation.w);
    dualArmRobot.moveObject("box7", box7_goal_pose_stamped, 0.1);

    sleep(1);
    left_rot.DoRotX(3.14 /12);
    left_rot.GetQuaternion(box7_goal_pose_stamped.pose.orientation.x, box7_goal_pose_stamped.pose.orientation.y, box7_goal_pose_stamped.pose.orientation.z,
                           box7_goal_pose_stamped.pose.orientation.w);
    dualArmRobot.moveObject("box7", box7_goal_pose_stamped, 0.1);
    sleep(1);
    left_rot.DoRotY(3.14 /12);
    left_rot.GetQuaternion(box7_goal_pose_stamped.pose.orientation.x, box7_goal_pose_stamped.pose.orientation.y, box7_goal_pose_stamped.pose.orientation.z,
                           box7_goal_pose_stamped.pose.orientation.w);

    left_rot.GetEulerZYX(alfa, beta, gamma);
    ROS_INFO("After alfa = %f\tbeta = %f\t gamma = %f", alfa, beta, gamma);
    dualArmRobot.moveObject("box7", box7_goal_pose_stamped, 0.1);
    sleep(1);

    left_rot.DoRotY(-3.14 /12);
    left_rot.GetQuaternion(box7_goal_pose_stamped.pose.orientation.x, box7_goal_pose_stamped.pose.orientation.y, box7_goal_pose_stamped.pose.orientation.z,
                           box7_goal_pose_stamped.pose.orientation.w);
    dualArmRobot.moveObject("box7", box7_goal_pose_stamped, 0.1);
    sleep(1);
    left_rot.DoRotZ(-3.14 / 18);
    left_rot.GetQuaternion(box7_goal_pose_stamped.pose.orientation.x, box7_goal_pose_stamped.pose.orientation.y, box7_goal_pose_stamped.pose.orientation.z,
                           box7_goal_pose_stamped.pose.orientation.w);
    dualArmRobot.moveObject("box7", box7_goal_pose_stamped, 0.1);

    sleep(1);
    left_rot.DoRotZ(3.14 / 18);
    left_rot.GetQuaternion(box7_goal_pose_stamped.pose.orientation.x, box7_goal_pose_stamped.pose.orientation.y, box7_goal_pose_stamped.pose.orientation.z,
                           box7_goal_pose_stamped.pose.orientation.w);
    dualArmRobot.moveObject("box7", box7_goal_pose_stamped, 0.1);
    sleep(1);

    // // Create an desired frame
    // KDL::Frame desired_end_effector_pose(
    // KDL::Rotation::RPY(-1.57,0,1.57), // Rotation rad
    // KDL::Vector(-0.2,-0.3,0.8));      // Position x,y,z in meters
    // KDL::Rotation rot = left_frame_eef.R

    // // clear constraints of ur5
    // dualArmRobot.left_.clearPathConstraints();
    ROS_INFO("========== PLACE DOWN =================");
    // Place box7
    geometry_msgs::Vector3 go_down;
    go_down.x = 0;
    go_down.y = 0;
    go_down.z = -0.15;
    if (!dualArmRobot.placeBox("box7", box7_goal_pose_stamped, go_down))
    {
        ROS_WARN("Place Box failed");
        ROS_ERROR("Demonstration aborted to avoid further problems");
        return 0;
    }

    // evaluation
    after_place_7 = ros::Time::now();
    manipulation_7 = after_place_7 - before_pick_7;
    ROS_INFO(":::::: VALUES EVALUATION ::::::");
    ROS_INFO("manipulation box 7 took: %li nsec", manipulation_7.toNSec());
    sleep(5);

    dualArmRobot.moveHome();
    /*
    // setup constraints
    // ur5 sometimes blocks itself when moving the box on bottom, this should solve the issue
    left_constraints.joint_constraints.clear();
    right_constraints.joint_constraints.clear();
    both_constraints.joint_constraints.clear();

    jcm.joint_name="left_wrist_2_joint";
    jcm.position = 1.5;
    jcm.tolerance_above = 0.6;
    jcm.tolerance_below = 0.6;
    jcm.weight = 1.0;
    left_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.left_.setPathConstraints(left_constraints);
    both_constraints.joint_constraints.push_back(jcm);

    // ur5 sometimes blocks itself for path adaption when picking the box on bottom, this should solve the issue
    jcm.joint_name="right_wrist_1_joint";
    jcm.position = 0.7;
    jcm.tolerance_above = 1.0;
    jcm.tolerance_below = 1.0;
    jcm.weight = 1.0;
    right_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.right_.setPathConstraints(right_constraints);
    both_constraints.joint_constraints.push_back(jcm);

    // when placing box on top ur5 can get blocked because wrist 1 reaches limit
    jcm.joint_name="left_wrist_1_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 3.0;
    jcm.tolerance_below = 3.0;
    jcm.weight = 1.0;
    left_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.left_.setPathConstraints(left_constraints);
    both_constraints.joint_constraints.push_back(jcm);

    jcm.joint_name="left_shoulder_pan_joint";
    jcm.position = -2.4;
    jcm.tolerance_above = 2.4;
    jcm.tolerance_below = 0.7;
    jcm.weight = 1.0;
    left_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.left_.setPathConstraints(left_constraints);
    both_constraints.joint_constraints.push_back(jcm);

    // ur5 sometimes blocks itself when picking the box on top, this should solve the issue
    jcm.joint_name="right_wrist_2_joint";
    jcm.position = 0;
    jcm.tolerance_above = 2.5;
    jcm.tolerance_below = 2.5;
    jcm.weight = 1.0;
    right_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.right_.setPathConstraints(right_constraints);
    both_constraints.joint_constraints.push_back(jcm);

    dualArmRobot.arms_.setPathConstraints(both_constraints);


    //Eval
    ros::Time before_pick_3;
    ros::Duration manipulation_3;
    ros::Time after_place_3;

    before_pick_3 = ros::Time::now();

    // Pick box3 on bottom
    geometry_msgs::Vector3Stamped direction2;
    direction2.header.frame_id = "world";
    direction2.vector.x = 0.20;
    direction2.vector.y = 0.20;
    direction2.vector.z = 0.02;
    if (!dualArmRobot.pickBox("box3", direction2)) {
        ROS_WARN("Pick failed");
        ROS_ERROR("Can't execute demonstration without successful pick. Demonstration aborted.");
        return 0;
    }

    // Place box3 by pushing it to its goal position
    // first, determine box3 goal pose
    geometry_msgs::Pose box3_goal_pose;
    box3_goal_pose.position.x = 0.04 + sceneManager.box_.dimensions[1]/2 +0.001; //+ sceneManager.box_.dimensions[1]
    box3_goal_pose.position.y = 0.04 + sceneManager.box_.dimensions[2]/2 + sceneManager.box_.dimensions[2]+0.001;
    box3_goal_pose.position.z = 0.0155+0.52/2+0.0155/2+sceneManager.box_.dimensions[0]/2+0.005;
    KDL::Rotation box3_goal_rot;
    box3_goal_rot.DoRotY(-3.14/2);
    box3_goal_rot.GetQuaternion(box3_goal_pose.orientation.x, box3_goal_pose.orientation.y, box3_goal_pose.orientation.z, box3_goal_pose.orientation.w);
    geometry_msgs::PoseStamped box3_goal_pose_stamped;
    box3_goal_pose_stamped.header.frame_id = "shelf";
    box3_goal_pose_stamped.pose=box3_goal_pose;

    // second, determine direction for pushing box into goal position
    geometry_msgs::Vector3 direction_push;
    direction_push.x = 0.0;
    direction_push.y = - (sceneManager.box_.dimensions[2]+0.005);// + 0.04);
    direction_push.z = 0.0;

    // setup constraints
    dualArmRobot.left_.clearPathConstraints();

    // pre-goal position
    ROS_INFO("moving closer to target position");
    geometry_msgs::PoseStamped left_pre_pose = dualArmRobot.left_current_pose_;
    left_pre_pose.pose.position.z += 0.2;
    if (!dualArmRobot.moveObject("box3", left_pre_pose, 0.5)){
        ROS_ERROR("Failed to move box3. Demonstration aborted.");
        return false;
    }

    // start Push Place sequence
    if (!dualArmRobot.pushPlaceBox("box3", box3_goal_pose_stamped, direction_push)){
        ROS_WARN("Push Place Box failed");
        ROS_ERROR("Demonstration aborted to avoid further problems");
        return  0;
    }

    // evaluation
    after_place_3 = ros::Time::now();
    manipulation_3 = after_place_3 - before_pick_3;
    ROS_INFO(":::::: VALUES EVALUATION ::::::");
    ROS_INFO("manipulation box 3 took: %li nsec", manipulation_3.toNSec());
    sleep(5);
    
    // move robot back into home pose
    dualArmRobot.right_.clearPathConstraints();
    dualArmRobot.left_.clearPathConstraints();
    dualArmRobot.moveHome();
*/
    // END
    ROS_INFO("Finished demonstration");
    sleep(1);
    ur_logger.stop();
    ros::shutdown();
    return 0;
}