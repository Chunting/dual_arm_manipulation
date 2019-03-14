//
// Created by Chunting .
//
// Dual Arm Toolbox
#include "dual_arm_toolbox/TrajectoryProcessor.h"
#include "dual_arm_toolbox/Transform.h"

#include <moveit/move_group_interface/move_group.h>
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
void run_experiment(ros::NodeHandle &nh, ros::Publisher &left_speed_pub, moveit::planning_interface::MoveGroup &left_, double velocity, double moving_distance){
    // create ur_logger. Use this namespace
    std::vector<std::string> ur_namespaces;
    ur_namespaces.push_back("");
    UR_Logger ur_logger(nh, ur_namespaces);

    // stop controller
    ros::ServiceClient left_srv_switch_controller = nh.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
    controller_manager_msgs::SwitchController srv_req;
    srv_req.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
    srv_req.request.stop_controllers.push_back("vel_based_pos_traj_controller");
    bool success = left_srv_switch_controller.call(srv_req);
    ROS_INFO("Stopping controller %s",success?"SUCCEDED":"FAILED");
    srv_req.request.stop_controllers.clear();

    geometry_msgs::PoseStamped current_pose = left_.getCurrentPose();
    std::cout << "x: " << current_pose.pose.position.x << "\ty: " << current_pose.pose.position.y << "\tz:  " << current_pose.pose.position.z << std::endl;
    double radius = sqrt(current_pose.pose.position.x*current_pose.pose.position.x + current_pose.pose.position.y*current_pose.pose.position.y);
    std::cout << "RADIUS: " << radius << std::endl;
    double omega = 0.1;//velocity/radius;
    double moving_time = moving_distance/velocity; //move 3cm, obstacle is around 1-2 cm away.

    // Output Message Speed = const
    trajectory_msgs::JointTrajectory joint_traj; //containing speed command
    trajectory_msgs::JointTrajectoryPoint traj_point;
    traj_point.velocities.assign(6,0);
    traj_point.velocities[0] = -omega;
    joint_traj.points.push_back(traj_point);

    // start logging
    ur_logger.start(50);

    ROS_INFO("Publishing velocity commands to left_ at 100Hz");

    // publish messages
    ros::Rate loop_rate(100);   // velocity-message publish rate

    Stopwatch stopwatch;

    while (ros::ok() && (stopwatch.elapsed().toSec()<moving_time))
    {
        joint_traj.header.stamp = ros::Time::now();
        left_speed_pub.publish(joint_traj);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("setting vel to 0");
    joint_traj.points.pop_back();
    traj_point.velocities[0] = 0.0;
    joint_traj.points.push_back(traj_point);

    stopwatch.restart();

    while (ros::ok() && (stopwatch.elapsed().toSec()<1.5))
    {
        joint_traj.header.stamp = ros::Time::now();
        left_speed_pub.publish(joint_traj);

        ros::spinOnce();
        loop_rate.sleep();
    }

    joint_traj.points.pop_back();
    traj_point.velocities[0] = omega;
    joint_traj.points.push_back(traj_point);

    ROS_INFO("moving back");
    stopwatch.restart();

    while (ros::ok() && (stopwatch.elapsed().toSec()<moving_time))
    {
        joint_traj.header.stamp = ros::Time::now();
        left_speed_pub.publish(joint_traj);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Stopped publishing");

    // stop logging
    ur_logger.stop();

    // restart controller
    srv_req.request.BEST_EFFORT;
    srv_req.request.start_controllers.push_back("vel_based_pos_traj_controller");
    success = left_srv_switch_controller.call(srv_req);
    ROS_INFO("Starting controller %s",success?"SUCCEDED":"FAILED");
    srv_req.request.start_controllers.clear();
}


int main(int argc, char **argv)
{
    // ROS Setup
    ros::init(argc, argv, "ur_const_vel_publisher");
    ros::NodeHandle nh;
    ros::Publisher left_speed_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/left/ur_driver/joint_speed", 1);
    ros::AsyncSpinner asyncSpinner(2);
    asyncSpinner.start();

    // MoveGroup
    moveit::planning_interface::MoveGroup left_("left_manipulator");
    moveit::planning_interface::MoveGroup::Plan plan;
    left_.setPlanningTime(30);

    ROS_INFO("Reference Frame: %s", left_.getPlanningFrame().c_str());
    ROS_INFO("EndEffectorLink: %s", left_.getEndEffectorLink().c_str());
     // left_joint_values = getJointAngles("left_manipulator");
    geometry_msgs::PoseStamped left_current_pose_ = left_.getCurrentPose(left_.getEndEffectorLink());
    ROS_INFO("\nleft_current_pose_ frame_id: %s, end_effector: %s, x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f\n", 
        left_current_pose_.header.frame_id.c_str()
        ,left_.getEndEffectorLink().c_str()
        ,left_current_pose_.pose.position.x
        ,left_current_pose_.pose.position.y
        ,left_current_pose_.pose.position.z
        ,left_current_pose_.pose.orientation.x
        ,left_current_pose_.pose.orientation.y
        ,left_current_pose_.pose.orientation.z
        ,left_current_pose_.pose.orientation.w);

    std::vector<std::string> leftJointNames = left_.getActiveJoints();
    std::vector<double> leftJointValues = left_.getCurrentJointValues();
    for(std::size_t i=0; i<leftJointNames.size(); i++){
         ROS_INFO("Joint %s: %f", leftJointNames[i].c_str(), leftJointValues[i]);
    }

    ROS_WARN("robot is moving without collision checking. BE CAREFUL!");
    ROS_INFO("waiting 10 Seconds. Press Ctrl-C if Robot is in the wrong start position");
    ros::Duration(5).sleep();

    //long distance position
    /*
    left_.setJointValueTarget("elbow_joint", -1.366541959239651);
    left_.setJointValueTarget("shoulder_lift_joint", -2.573810648739345);
    left_.setJointValueTarget("shoulder_pan_joint", 0.5943102022167164);
    left_.setJointValueTarget("wrist_1_joint", -0.7533539281232803);
    left_.setJointValueTarget("wrist_2_joint", -0.0);
    left_.setJointValueTarget("wrist_3_joint", 0.00015758105264953662);*/

    // short distance postion
    left_.setJointValueTarget("left_shoulder_pan_joint", -1.1800545151007107);
    left_.setJointValueTarget("left_elbow_joint", -1.9646600642315206);
    left_.setJointValueTarget("left_shoulder_lift_joint", -2.2494529549924893);
    left_.setJointValueTarget("left_wrist_1_joint", -0.41466027950402257);
    left_.setJointValueTarget("left_wrist_2_joint", -0.0);
    left_.setJointValueTarget("left_wrist_3_joint", 0.00300112795031922);

    left_.plan(plan);

    ros::Publisher execTrajectoryPub_ = nh.advertise<moveit_msgs::RobotTrajectory>("/execute_my_trajectory", 1, true);
    moveit_msgs::RobotTrajectory trajectory_ = plan.trajectory_;
    // ROS_INFO("Publishing plan and waiting for %i seconds", sec);
    execTrajectoryPub_.publish(trajectory_);
    sleep(1);
    // ROS_INFO("Header time  %f ", trajectory_.joint_trajectory.header.stamp.toSec());
    //     for (unsigned int i = 0; i < trajectory_.joint_trajectory.points.size(); i++){
    //         ROS_INFO("Listening Points %d  %f ", i, trajectory_.joint_trajectory.header.stamp.toSec()+trajectory_.joint_trajectory.points[i].time_from_start.toSec());
    //         for (unsigned int a = 0; a < trajectory_.joint_trajectory.points[i].positions.size(); a++){
    //             ROS_INFO("%s:\tpos %f\tvel %f", 
    //             trajectory_.joint_trajectory.joint_names[a].c_str(), 
    //             trajectory_.joint_trajectory.points[i].positions[a]*(180/3.14159),
    //             trajectory_.joint_trajectory.points[i].velocities[a]*(180/3.14159));
    //         }
    //     }

    ROS_WARN("visualizing plan. STRG+C to interrupt.");
    sleep(4);
    left_.execute(plan);
    sleep(3);

    // ::::::: Run Experiments :::::::
/*
    ROS_INFO("::: Experiment 1 ::: vel = 0.02m/s; s = 0.03m");
    run_experiment(nh, left_speed_pub, left_, 0.02, 0.03);

    ROS_INFO("moveing to start");
    left_.plan(plan);
    left_.execute(plan);

    ROS_INFO("::: Experiment 2 ::: vel = 0.01m/s; s = 0.03m");
    run_experiment(nh, left_speed_pub, left_, 0.01, 0.03);

    ROS_INFO("moveing to start");
    left_.plan(plan);
    left_.execute(plan);

    ROS_INFO("::: Experiment 3 ::: vel = 0.005m/s; s = 0.03m");
    run_experiment(nh, left_speed_pub, left_, 0.005, 0.03);

    ROS_INFO("moveing to start");
    left_.plan(plan);
    left_.execute(plan);
*/
    // ROS_INFO("::: Experiment 4 ::: vel = 0.001m/s; s = 0.03m");
    // run_experiment(nh, left_speed_pub, left_, 0.001, 0.03);

    // ROS_INFO("moveing to start");
    // left_.plan(plan);
    // left_.execute(plan);


    ROS_INFO("finished. shutting down.");

    ros::shutdown();
    return 0;
}
