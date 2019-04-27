//
// Created by Chunting .
// rosrun dual_arm_robot_applications dual_arm_robot_vel_force_experiment
// Dual Arm Toolbox
#include "dual_arm_toolbox/TrajectoryProcessor.h"
#include "dual_arm_toolbox/Transform.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/SwitchControllerRequest.h>
#include <controller_manager_msgs/SwitchControllerResponse.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include "ur_logging/UrLogger.h"

/* TODO
 * vel increase, fall to zero, move backwards
 */

// m/s ; m
void run_experiment(ros::NodeHandle &nh, ros::Publisher &c_joint_speed_pub,
                    moveit::planning_interface::MoveGroupInterface &left_,
                    double velocity, double moving_distance)
{
    ros::Time k = ros::Time::now();
    ///////////////////////////////////////////////////////////////////////////////////
    // stop controller
    ros::ServiceClient left_srv_switch_controller = nh.serviceClient<controller_manager_msgs::SwitchController>("/left/controller_manager/switch_controller");
    controller_manager_msgs::SwitchController srv_req;
    srv_req.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
    srv_req.request.stop_controllers.push_back("/left/vel_based_pos_traj_controller");
    bool success = left_srv_switch_controller.call(srv_req);
    ROS_INFO("Stopping controller %s", success ? "SUCCEDED" : "FAILED");
    srv_req.request.stop_controllers.clear();
    //////////////////////////////////////////////////////////////////////////////////////
    geometry_msgs::PoseStamped current_pose = left_.getCurrentPose();
    std::cout << "x: " << current_pose.pose.position.x << "\ty: " << current_pose.pose.position.y << "\tz:  " << current_pose.pose.position.z << std::endl;
    double radius = sqrt(current_pose.pose.position.x * current_pose.pose.position.x + current_pose.pose.position.y * current_pose.pose.position.y);
    double omega = velocity / radius;                //velocity/radius;
    double moving_time = moving_distance / velocity; //move 3cm, obstacle is around 1-2 cm away.
    std::cout << "RADIUS: " << radius << "\t omega: " << omega << "\tmoving_time: " << moving_time << std::endl;
    // Output Message Speed = const
    trajectory_msgs::JointTrajectory joint_traj; //containing speed command
    trajectory_msgs::JointTrajectoryPoint traj_point;
    std::vector<std::string> leftJointNames = left_.getActiveJoints();
    // std::vector<double> leftJointValues = left_.getCurrentJointValues();
    for (std::size_t i = 0; i < leftJointNames.size(); i++)
    {
        joint_traj.joint_names.push_back(leftJointNames[i]);
    }
    traj_point.velocities.assign(6, 0);
    traj_point.velocities[5] = -omega;
    joint_traj.points.push_back(traj_point);
   
  

    // publish messages
    ros::Rate loop_rate(50); // velocity-message publish rate

    Stopwatch stopwatch;

    while (ros::ok() && (stopwatch.elapsed().toSec() < 1.5))
    {
        joint_traj.header.stamp = ros::Time::now();
        c_joint_speed_pub.publish(joint_traj);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("setting vel to 0");
    joint_traj.points.pop_back();
    traj_point.velocities[5] = 0.0;
    joint_traj.points.push_back(traj_point);

    stopwatch.restart();

    while (ros::ok() && (stopwatch.elapsed().toSec() < 1.5))
    {
        joint_traj.header.stamp = ros::Time::now();
        c_joint_speed_pub.publish(joint_traj);

        ros::spinOnce();
        loop_rate.sleep();
    }

    joint_traj.points.pop_back();
    traj_point.velocities[5] = omega;
    joint_traj.points.push_back(traj_point);

    ROS_INFO("moving back");
    stopwatch.restart();

    while (ros::ok() && (stopwatch.elapsed().toSec() < 1.5))
    {
        joint_traj.header.stamp = ros::Time::now();
        c_joint_speed_pub.publish(joint_traj);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Stopped publishing");

    // restart controller
    srv_req.request.BEST_EFFORT;
    srv_req.request.start_controllers.push_back("/left/vel_based_pos_traj_controller");
    success = left_srv_switch_controller.call(srv_req);
    ROS_INFO("Starting controller %s", success ? "SUCCEDED" : "FAILED");
    srv_req.request.start_controllers.clear();
}

int main(int argc, char **argv)
{
    // ROS Setup
    ros::init(argc, argv, "ur_const_vel_publisher");
    ros::NodeHandle nh;
    ros::Publisher c_joint_speed_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/left/ur_driver/joint_speed", 1);

    //  ros::Subscriber sub = nh.subscribe("/joint_states", 1, chatterCallback);
    ros::Publisher m_pub_tcp_speed = nh.advertise<geometry_msgs::TwistStamped>("/left/m_tool_velocity", 1);
    ros::Publisher c_pub_tcp_speed = nh.advertise<geometry_msgs::TwistStamped>("/left/c_tool_velocity", 1);

    ros::AsyncSpinner asyncSpinner(2);
    asyncSpinner.start();

    // MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface left_("left_manipulator");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    left_.setPlanningTime(30);

    ROS_INFO("Reference Frame: %s", left_.getPlanningFrame().c_str());
    ROS_INFO("EndEffectorLink: %s", left_.getEndEffectorLink().c_str());
    // left_joint_values = getJointAngles("left_manipulator");
    geometry_msgs::PoseStamped left_current_pose_ = left_.getCurrentPose(left_.getEndEffectorLink());
    ROS_INFO("\nleft_current_pose_ frame_id: %s, end_effector: %s\n x=%f, y=%f, z=%f\n qx=%f, qy=%f, qz=%f, qw=%f\n",
             left_current_pose_.header.frame_id.c_str(), left_.getEndEffectorLink().c_str(), left_current_pose_.pose.position.x, left_current_pose_.pose.position.y, left_current_pose_.pose.position.z, left_current_pose_.pose.orientation.x, left_current_pose_.pose.orientation.y, left_current_pose_.pose.orientation.z, left_current_pose_.pose.orientation.w);

    std::vector<std::string> leftJointNames = left_.getActiveJoints();
    std::vector<double> leftJointValues = left_.getCurrentJointValues();
    for (std::size_t i = 0; i < leftJointNames.size(); i++)
    {
        ROS_INFO("Joint %s:\t%f", leftJointNames[i].c_str(), leftJointValues[i]);
    }

    ROS_WARN("robot is moving without collision checking. BE CAREFUL!");
    ROS_INFO("waiting 10 Seconds. Press Ctrl-C if Robot is in the wrong start position");
    ros::Duration(5).sleep();

    // short distance postion
    left_.setJointValueTarget("left_shoulder_pan_joint", -1.57);
    left_.setJointValueTarget("left_elbow_joint", -1.57);
    left_.setJointValueTarget("left_shoulder_lift_joint", 0.785);
    left_.setJointValueTarget("left_wrist_1_joint", -1.57);
    left_.setJointValueTarget("left_wrist_2_joint", -1.57);
    left_.setJointValueTarget("left_wrist_3_joint", 1.57);

    // left_.plan(plan);

    // ros::Publisher execTrajectoryPub_ = nh.advertise<moveit_msgs::RobotTrajectory>("/execute_my_trajectory", 1, true);
    // moveit_msgs::RobotTrajectory trajectory_ = plan.trajectory_;
    // ROS_INFO("Publishing plan and waiting for %i seconds", sec);
    // execTrajectoryPub_.publish(trajectory_);
    // sleep(1);
    // ROS_WARN("visualizing plan. STRG+C to interrupt.");
    // sleep(4);
    // left_.execute(plan);
    // sleep(3);

    // ::::::: Run Experiments :::::::

    ROS_INFO("::::::::::::::::: Experiment 1 ::: vel = 0.2m/s; s = 0.2m");
    run_experiment(nh, c_joint_speed_pub, left_, 0.02, 0.05);
    sleep(1);
    // ROS_INFO("moveing to start");
    // left_.plan(plan);
    // left_.execute(plan);
    sleep(1);
    ROS_INFO("::::::::::::::::: Experiment 2 ::: vel = 0.1m/s; s = 0.2m");
    run_experiment(nh, c_joint_speed_pub, left_, 0.01, 0.05);
    sleep(1);
    ROS_INFO("moveing to start");
    // left_.plan(plan);
    // left_.execute(plan);
    sleep(1);
    ROS_INFO("::::::::::::::::: Experiment 3 ::: vel = 0.05m/s; s = 0.05m");
    run_experiment(nh, c_joint_speed_pub, left_, 0.005, 0.05);
    sleep(1);
    ROS_INFO("moveing to start");
    // left_.plan(plan);
    // left_.execute(plan);
    sleep(1);
    ROS_INFO("::::::::::::::::: Experiment 4 ::: vel = 0.01m/s; s = 0.05m");
    run_experiment(nh, c_joint_speed_pub, left_, 0.001, 0.05);
    sleep(1);
    ROS_INFO("moveing to start");
    // left_.plan(plan);
    // left_.execute(plan);
    sleep(1);
    ROS_INFO("finished. shutting down.");
    ros::shutdown();
    return 0;
}
