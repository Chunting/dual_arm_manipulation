//
// Created by Chunting  on 07.10.16.
// rosrun dual_arm_robot_applications dual_arm_robot_demonstration

// ROS
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/PlanningScene.h>

// Rviz
#include <moveit_msgs/DisplayTrajectory.h>

// Dual Arm Tools
#include "dual_arm_toolbox/TrajectoryProcessor.h"
#include "dual_arm_toolbox/Transform.h"

// Dual Arm Demonstrator
#include "dual_arm_demonstrator_iml/DualArmRobot.h"
#include "dual_arm_demonstrator_iml/SceneManager.h"

// #include "dual_arm_demonstrator_iml/FTSensorSubscriber.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dual_arm_robot_demonstration");
  ros::AsyncSpinner spinner(4);
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
  // Force torque sensor setup
  FTSensorSubscriber left_robotiq_ft_subscriber(nh, "left");
  FTSensorSubscriber right_robotiq_ft_subscriber(nh, "right");

  geometry_msgs::Vector3Stamped direction;
  direction.header.frame_id = "world";
  dualArmRobot.setConstraints();
  dualArmRobot.kinematic_statePtr->enforceBounds();
  // Cartesian velocity controller command topic
  std::string topic_left_arm_cmd("/left/ur5_cartesian_velocity_controller/command_cart_vel");
  ros::Publisher pub_left_arm_cmd_ = nh.advertise<geometry_msgs::Twist>(topic_left_arm_cmd, 5);
  std::string topic_right_arm_cmd("/right/ur5_cartesian_velocity_controller/command_cart_vel");
  ros::Publisher pub_right_arm_cmd_ = nh.advertise<geometry_msgs::Twist>(topic_right_arm_cmd, 5);

  ros::Rate loop_rate(100);
  Matrix6d rotation_left_base_world;
  Matrix6d rotation_right_base_world;
  Matrix6d rotation_world_left_ee;
  Matrix6d rotation_world_right_ee;

  rotation_world_left_ee.setZero();
  rotation_world_right_ee.setZero();
  bool left_base_world_ready_ = false;
  bool right_base_world_ready_ = false;
  tf::TransformListener listener_arm_;
  // while (!dualArmRobot.get_rotation_matrix(rotation_left_base_world, listener_arm_, "left_base_link", "world"))
  // {
  //   sleep(1);
  // }
  // left_base_world_ready_ = true;
  // while (!dualArmRobot.get_rotation_matrix(rotation_right_base_world, listener_arm_, "right_base_link", "world"))
  // {
  //   sleep(1);
  // }
  right_base_world_ready_ = true;
  // if (!dualArmRobot.switch_controller("ur5_cartesian_velocity_controller", "vel_based_pos_traj_controller", "left"))
  //     ROS_WARN("failed switching controller");
  ROS_INFO("========== MOVE HOME POSITION =================");
  dualArmRobot.moveHome();
  sleep(1);

  ROS_INFO("========== MOVE GRASP POSITION =================");
  dualArmRobot.moveGraspPosition();
  sleep(1);

  ROS_INFO("========== MOVE CLOSER =================");

  // dualArmRobot.graspMove(0.01, false, true, false);
  if (!dualArmRobot.switch_controller("vel_based_pos_traj_controller", "ur5_cartesian_velocity_controller", "left"))
    ROS_WARN("Failed switching left controller");
  if (!dualArmRobot.switch_controller("vel_based_pos_traj_controller", "ur5_cartesian_velocity_controller", "right"))
    ROS_WARN("Failed switching right controller");
  sleep(2);
  double left_force_norm = left_robotiq_ft_subscriber.wrench_external_(2);
  double right_force_norm = right_robotiq_ft_subscriber.wrench_external_(2);
  double desired_diff_ = std::abs(left_force_norm - right_force_norm);

  /* Adjust left arm pose */
  geometry_msgs::Twist left_arm_vel_cmd;
  geometry_msgs::Twist right_arm_vel_cmd;
  Vector6d world_left_arm_cmd_vel_;
  Vector6d world_right_arm_cmd_vel_;
  Vector6d left_base_arm_cmd_vel_;
  Vector6d right_base_arm_cmd_vel_;
  world_left_arm_cmd_vel_.setZero();
  world_right_arm_cmd_vel_.setZero();
  left_base_arm_cmd_vel_.setZero();
  right_base_arm_cmd_vel_.setZero();
  world_left_arm_cmd_vel_ << 0, 0.001, 0, 0, 0, 0;
  world_right_arm_cmd_vel_ << 0, -0.001, 0, 0, 0, 0;
  sleep(2);
  while (std::abs(left_force_norm) < 20 || desired_diff_ > 0.5)
  {
    left_base_arm_cmd_vel_ = rotation_left_base_world * world_left_arm_cmd_vel_;
    right_base_arm_cmd_vel_ = rotation_right_base_world * world_right_arm_cmd_vel_;

    dual_arm_toolbox::Transform::transformVector6dtoTwist(left_base_arm_cmd_vel_, left_arm_vel_cmd);
    dual_arm_toolbox::Transform::transformVector6dtoTwist(right_base_arm_cmd_vel_, right_arm_vel_cmd);
    pub_left_arm_cmd_.publish(left_arm_vel_cmd);
    pub_right_arm_cmd_.publish(right_arm_vel_cmd);

    left_force_norm = left_robotiq_ft_subscriber.wrench_external_(2);
    right_force_norm = right_robotiq_ft_subscriber.wrench_external_(2);
    desired_diff_ = std::abs(left_force_norm - right_force_norm);
    ROS_INFO_STREAM("left_force_norm = " << left_force_norm << "  right_force_norm = " << right_force_norm
                                         << "\ndesired_diff_ = " << desired_diff_);
    ros::spinOnce();
    loop_rate.sleep();
  }
  left_base_arm_cmd_vel_.setZero();
  right_base_arm_cmd_vel_.setZero();
  dual_arm_toolbox::Transform::transformVector6dtoTwist(left_base_arm_cmd_vel_, left_arm_vel_cmd);
  dual_arm_toolbox::Transform::transformVector6dtoTwist(right_base_arm_cmd_vel_, right_arm_vel_cmd);
  pub_left_arm_cmd_.publish(left_arm_vel_cmd);
  pub_right_arm_cmd_.publish(right_arm_vel_cmd);


  
  ros::Publisher offset_desired_pub = nh.advertise<geometry_msgs::PointStamped>("/desired_offset_point", 1);
  // Publish the desired offset between two EEs, described in right EE coordinate system
  KDL::Frame desired_offset = dualArmRobot.getCurrentOffset();  // w.r.t left_ee_link coordinate system
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
  ros::Duration duration(0);
  ros::Time after_place_7;

  
  // Pick box7 on top
  direction.header.frame_id = "world";
  direction.vector.x = 0;
  direction.vector.y = 0;
  direction.vector.z = 0.15;
  world_left_arm_cmd_vel_.setZero();
  world_right_arm_cmd_vel_.setZero();
  left_base_arm_cmd_vel_.setZero();
  right_base_arm_cmd_vel_.setZero();
  world_left_arm_cmd_vel_ << 0, 0, 0.01, 0, 0, 0;
  world_right_arm_cmd_vel_ << 0, 0, 0.01, 0, 0, 0;
  
 
  sleep(2);
  before_pick_7 = ros::Time::now();
  while (duration.toSec() < 10 && nh.ok())
  {
    
    double temp_diff = std::abs(left_robotiq_ft_subscriber.wrench_external_(2) - right_robotiq_ft_subscriber.wrench_external_(2));
    if(std::abs(desired_diff_ - temp_diff)>0.5){
        world_right_arm_cmd_vel_(1) = 0.002*(desired_diff_ - temp_diff);
    }else {
        world_right_arm_cmd_vel_(1) = 0 ;
    }
    left_base_arm_cmd_vel_ = rotation_left_base_world * world_left_arm_cmd_vel_;
    right_base_arm_cmd_vel_ = rotation_right_base_world * world_right_arm_cmd_vel_;
    dual_arm_toolbox::Transform::transformVector6dtoTwist(left_base_arm_cmd_vel_, left_arm_vel_cmd);
    dual_arm_toolbox::Transform::transformVector6dtoTwist(right_base_arm_cmd_vel_, right_arm_vel_cmd);
    pub_left_arm_cmd_.publish(left_arm_vel_cmd);
    pub_right_arm_cmd_.publish(right_arm_vel_cmd);
    ros::spinOnce();
    loop_rate.sleep();
    // evaluation
    after_place_7 = ros::Time::now();
    duration = after_place_7 - before_pick_7;
    // ROS_INFO("manipulation box 7 took: %f sec", duration.toSec());
  }
  world_left_arm_cmd_vel_.setZero();
  world_right_arm_cmd_vel_.setZero();
  left_base_arm_cmd_vel_.setZero();
  right_base_arm_cmd_vel_.setZero();

  dual_arm_toolbox::Transform::transformVector6dtoTwist(left_base_arm_cmd_vel_, left_arm_vel_cmd);
  dual_arm_toolbox::Transform::transformVector6dtoTwist(right_base_arm_cmd_vel_, right_arm_vel_cmd);
  pub_left_arm_cmd_.publish(left_arm_vel_cmd);
  pub_right_arm_cmd_.publish(right_arm_vel_cmd);
  sleep(2);

  if (!dualArmRobot.switch_controller("ur5_cartesian_velocity_controller", "vel_based_pos_traj_controller", "left"))
    ROS_WARN("failed switching controller");
  if (!dualArmRobot.switch_controller("ur5_cartesian_velocity_controller", "vel_based_pos_traj_controller", "right"))
    ROS_WARN("failed switching controller");

  geometry_msgs::PoseStamped box7_goal_pose_stamped;
  box7_goal_pose_stamped.header = dualArmRobot.left_current_pose_.header;

  dualArmRobot.left_current_pose_ = dualArmRobot.left_.getCurrentPose(dualArmRobot.left_.getEndEffectorLink());
  box7_goal_pose_stamped.pose = dualArmRobot.left_current_pose_.pose;
  KDL::Frame left_frame_eef;  // endeffector frame
  dual_arm_toolbox::Transform::transformPoseToKDL(dualArmRobot.left_current_pose_.pose, left_frame_eef);
  KDL::Rotation left_rot = left_frame_eef.M;
  // KDL::Rotation rotationAgnle = KDL::Rotation::Identity();
  double yaw = 0;    // Z-axis
  double pitch = 0;  // Y-axis
  double roll = 0;   // X-axis
  double angle = 0;
  left_rot.GetEulerZYX(yaw, pitch, roll);
  ROS_INFO("Before roll = %f\tpitch = %f\t yaw = %f", roll, pitch, yaw);
  angle = 3.14 / 12;
  pitch += angle;
  left_rot = KDL::Rotation::RPY(roll, pitch, yaw);

  left_rot.GetEulerZYX(yaw, pitch, roll);
  left_rot.GetQuaternion(box7_goal_pose_stamped.pose.orientation.x, box7_goal_pose_stamped.pose.orientation.y,
                         box7_goal_pose_stamped.pose.orientation.z, box7_goal_pose_stamped.pose.orientation.w);
  ROS_INFO("After roll = %f\tpitch = %f\t yaw = %f", roll, pitch, yaw);
  dualArmRobot.moveObject("box7", box7_goal_pose_stamped, 0.1);
  sleep(5);

  pitch -= angle;
  left_rot = KDL::Rotation::RPY(roll, pitch, yaw);
  left_rot.GetEulerZYX(yaw, pitch, roll);
  ROS_INFO("After roll = %f\tpitch = %f\t yaw = %f", roll, pitch, yaw);

  left_rot.GetQuaternion(box7_goal_pose_stamped.pose.orientation.x, box7_goal_pose_stamped.pose.orientation.y,
                         box7_goal_pose_stamped.pose.orientation.z, box7_goal_pose_stamped.pose.orientation.w);
  dualArmRobot.moveObject("box7", box7_goal_pose_stamped, 0.1);
  sleep(1);



  // dualArmRobot.graspMove(0.01, false, true, false);
  if (!dualArmRobot.switch_controller("vel_based_pos_traj_controller", "ur5_cartesian_velocity_controller", "left"))
    ROS_WARN("Failed switching left controller");
  if (!dualArmRobot.switch_controller("vel_based_pos_traj_controller", "ur5_cartesian_velocity_controller", "right"))
    ROS_WARN("Failed switching right controller");

  ROS_INFO("========== PLACE DOWN =================");
  world_left_arm_cmd_vel_ << 0, 0, -0.01, 0, 0, 0;
  world_right_arm_cmd_vel_ << 0, 0, -0.01, 0, 0, 0;
  duration = ros::Duration(0);
  before_pick_7 = ros::Time::now();
  while (duration.toSec() < 10 && nh.ok())
  {
    
    double temp_diff = std::abs(left_robotiq_ft_subscriber.wrench_external_(2) - right_robotiq_ft_subscriber.wrench_external_(2));
    if(std::abs(desired_diff_ - temp_diff)>1){
        world_right_arm_cmd_vel_(1) = 0.001*(desired_diff_ - temp_diff);
    }else {
        world_right_arm_cmd_vel_(1) = 0 ;
    }
    left_base_arm_cmd_vel_ = rotation_left_base_world * world_left_arm_cmd_vel_;
    right_base_arm_cmd_vel_ = rotation_right_base_world * world_right_arm_cmd_vel_;
    dual_arm_toolbox::Transform::transformVector6dtoTwist(left_base_arm_cmd_vel_, left_arm_vel_cmd);
    dual_arm_toolbox::Transform::transformVector6dtoTwist(right_base_arm_cmd_vel_, right_arm_vel_cmd);
    pub_left_arm_cmd_.publish(left_arm_vel_cmd);
    pub_right_arm_cmd_.publish(right_arm_vel_cmd);
    ros::spinOnce();
    loop_rate.sleep();
    // evaluation
    after_place_7 = ros::Time::now();
    duration = after_place_7 - before_pick_7;
    // ROS_INFO("manipulation box 7 took: %f sec", duration.toSec());
  }
  world_left_arm_cmd_vel_.setZero();
  world_right_arm_cmd_vel_.setZero();
  left_base_arm_cmd_vel_.setZero();
  right_base_arm_cmd_vel_.setZero();

  dual_arm_toolbox::Transform::transformVector6dtoTwist(left_base_arm_cmd_vel_, left_arm_vel_cmd);
  dual_arm_toolbox::Transform::transformVector6dtoTwist(right_base_arm_cmd_vel_, right_arm_vel_cmd);
  pub_left_arm_cmd_.publish(left_arm_vel_cmd);
  pub_right_arm_cmd_.publish(right_arm_vel_cmd);
  sleep(2);


  // if (!dualArmRobot.pickBox("box7", direction))
  // {
  // ROS_WARN("Pick failed");
  // ROS_ERROR("Can't execute demonstration without successful pick. Demonstration aborted.");
  // return 0;
  // }

  // box7 goal pose
  

/*
  // angle = 3.14/18;
  // pitch += angle;
  // left_rot = KDL::Rotation::RPY(roll, pitch, yaw);

  // left_rot.GetEulerZYX(yaw, pitch, roll);
  // left_rot.GetQuaternion(box7_goal_pose_stamped.pose.orientation.x,
  //                        box7_goal_pose_stamped.pose.orientation.y,
  //                        box7_goal_pose_stamped.pose.orientation.z,
  //                        box7_goal_pose_stamped.pose.orientation.w);
  // ROS_INFO("After roll = %f\tpitch = %f\t yaw = %f", roll, pitch, yaw);
  // dualArmRobot.moveObject("box7", box7_goal_pose_stamped, 0.1);
  // sleep(1);

  // pitch -= angle;
  // left_rot = KDL::Rotation::RPY(roll, pitch, yaw);
  // left_rot.GetEulerZYX(yaw, pitch, roll);
  // ROS_INFO("After roll = %f\tpitch = %f\t yaw = %f", roll, pitch, yaw);

  // left_rot.GetQuaternion(box7_goal_pose_stamped.pose.orientation.x,
  //                        box7_goal_pose_stamped.pose.orientation.y,
  //                        box7_goal_pose_stamped.pose.orientation.z,
  //                        box7_goal_pose_stamped.pose.orientation.w);
  // dualArmRobot.moveObject("box7", box7_goal_pose_stamped, 0.1);
  // sleep(1);

  // angle = -3.14 / 6;
  // yaw += angle;
  // left_rot = KDL::Rotation::RPY(roll, pitch, yaw);

  // left_rot.GetEulerZYX(yaw, pitch, roll);
  // left_rot.GetQuaternion(box7_goal_pose_stamped.pose.orientation.x,
  //                        box7_goal_pose_stamped.pose.orientation.y,
  //                        box7_goal_pose_stamped.pose.orientation.z,
  //                        box7_goal_pose_stamped.pose.orientation.w);
  // ROS_INFO("After roll = %f\tpitch = %f\t yaw = %f", roll, pitch, yaw);
  // dualArmRobot.moveObject("box7", box7_goal_pose_stamped, 0.1);
  // sleep(1);

  // yaw -= angle;
  // left_rot = KDL::Rotation::RPY(roll, pitch, yaw);
  // left_rot.GetEulerZYX(yaw, pitch, roll);
  // ROS_INFO("After roll = %f\tpitch = %f\t yaw = %f", roll, pitch, yaw);

  // left_rot.GetQuaternion(box7_goal_pose_stamped.pose.orientation.x,
  //                        box7_goal_pose_stamped.pose.orientation.y,
  //                        box7_goal_pose_stamped.pose.orientation.z,
  //                        box7_goal_pose_stamped.pose.orientation.w);
  // dualArmRobot.moveObject("box7", box7_goal_pose_stamped, 0.1);
  // sleep(1);

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
  duration = after_place_7 - before_pick_7;
  ROS_INFO(":::::: VALUES EVALUATION ::::::");
  ROS_INFO("manipulation box 7 took: %li nsec", duration.toSec());
  sleep(5);

  dualArmRobot.moveHome();

  // Eval
  ros::Time before_pick_3;
  ros::Duration manipulation_3;
  ros::Time after_place_3;

  before_pick_3 = ros::Time::now();
  /*
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
    box3_goal_rot.GetQuaternion(box3_goal_pose.orientation.x, box3_goal_pose.orientation.y,
    box3_goal_pose.orientation.z,
    box3_goal_pose.orientation.w);
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
    ROS_INFO("manipulation box 3 took: %li nsec", manipulation_3.toSec());
    sleep(5);
    */
  if (!dualArmRobot.switch_controller("ur5_cartesian_velocity_controller", "vel_based_pos_traj_controller", "left"))
    ROS_WARN("failed switching controller");
  if (!dualArmRobot.switch_controller("ur5_cartesian_velocity_controller", "vel_based_pos_traj_controller", "right"))
    ROS_WARN("failed switching controller");
  sleep(2);
  // move robot back into home pose
  dualArmRobot.right_.clearPathConstraints();
  dualArmRobot.left_.clearPathConstraints();
  dualArmRobot.moveHome();
  // END
  ROS_INFO("Finished demonstration");
  sleep(1);
  ros::shutdown();
  return 0;
}