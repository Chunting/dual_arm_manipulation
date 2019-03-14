//
// Created by Chunting  on 17.10.16.
//

#include "dual_arm_toolbox/TrajectoryProcessor.h"

using namespace dual_arm_toolbox;
// fuses two trajectories into one; both should be of same length
bool TrajectoryProcessor::fuse(moveit_msgs::RobotTrajectory &arms_trajectory,
                               moveit_msgs::RobotTrajectory arm1_trajectory,
                               moveit_msgs::RobotTrajectory arm2_trajectory) {
    arms_trajectory = arm1_trajectory;
    int i1 = arm1_trajectory.joint_trajectory.points.size();
    int i2 = arm2_trajectory.joint_trajectory.points.size();
    int num = std::min(i1,i2);
    if(i1<=i2){
        /* If arm1_trajectory has fewer points, remove the first points of arm2_trajectory,
           then the two arms have the same number of points to run*/
        num = i1;
        arms_trajectory = arm1_trajectory;
        for (unsigned int i=0; i < arm2_trajectory.joint_trajectory.joint_names.size(); i++)
        arms_trajectory.joint_trajectory.joint_names.push_back(arm2_trajectory.joint_trajectory.joint_names[i]);
        for (unsigned int i=0; i < num; i++){
            for (unsigned int j=0; j < arm2_trajectory.joint_trajectory.joint_names.size(); j++){
                int offset = i2-num;
                arms_trajectory.joint_trajectory.points[i].positions.push_back(arm2_trajectory.joint_trajectory.points[i+offset].positions[j]);
                arms_trajectory.joint_trajectory.points[i].accelerations.push_back(arm2_trajectory.joint_trajectory.points[i+offset].accelerations[j]);
                arms_trajectory.joint_trajectory.points[i].velocities.push_back(arm2_trajectory.joint_trajectory.points[i+offset].velocities[j]);
            }
        }
    } else {
        /* If arm2_trajectory has fewer points, remove the first points of arm1_trajectory
            then the two arms have the same number of points to run*/
        num = i2;
        for (unsigned int i=0; i < arm1_trajectory.joint_trajectory.joint_names.size(); i++)
            arms_trajectory.joint_trajectory.joint_names.push_back(arm1_trajectory.joint_trajectory.joint_names[i]);
        for (unsigned int i=0; i < arm2_trajectory.joint_trajectory.joint_names.size(); i++)
            arms_trajectory.joint_trajectory.joint_names.push_back(arm2_trajectory.joint_trajectory.joint_names[i]);
        for (unsigned int i=0; i < num; i++){
            int offset = i1-num;
            for (unsigned int j=0; j < arm1_trajectory.joint_trajectory.joint_names.size(); j++){
                arms_trajectory.joint_trajectory.points[i].positions.push_back(arm1_trajectory.joint_trajectory.points[i+offset].positions[j]);
                arms_trajectory.joint_trajectory.points[i].accelerations.push_back(arm1_trajectory.joint_trajectory.points[i+offset].accelerations[j]);
                arms_trajectory.joint_trajectory.points[i].velocities.push_back(arm1_trajectory.joint_trajectory.points[i+offset].velocities[j]);
            }
        }
        for (unsigned int i=0; i < num; i++){
            for (unsigned int j=0; j < arm2_trajectory.joint_trajectory.joint_names.size(); j++){
                arms_trajectory.joint_trajectory.points[i].positions.push_back(arm2_trajectory.joint_trajectory.points[i].positions[j]);
                arms_trajectory.joint_trajectory.points[i].accelerations.push_back(arm2_trajectory.joint_trajectory.points[i].accelerations[j]);
                arms_trajectory.joint_trajectory.points[i].velocities.push_back(arm2_trajectory.joint_trajectory.points[i].velocities[j]);
            }
        }
    }
    return true;
}
// splits one trajectory for both arms into two trajectory for each arm
// If the arms_trajectory only contains one arm's trajectory, the other arm's trajectory will be ignored by the comparison of arm prefix
bool TrajectoryProcessor::split(moveit_msgs::RobotTrajectory arms_trajectory,
                                moveit_msgs::RobotTrajectory &arm1_trajectory,
                                moveit_msgs::RobotTrajectory &arm2_trajectory,
                                std::string arm1_prefix,
                                std::string arm2_prefix) {
    // header
    arm1_trajectory.joint_trajectory.header = arms_trajectory.joint_trajectory.header;
    arm2_trajectory.joint_trajectory.header = arms_trajectory.joint_trajectory.header;

    // joint names
    for (unsigned int i = 0; i < arms_trajectory.joint_trajectory.joint_names.size(); i++){
        if (arms_trajectory.joint_trajectory.joint_names[i].compare(0,arm1_prefix.size(),arm1_prefix)==0){
            arm1_trajectory.joint_trajectory.joint_names.push_back(arms_trajectory.joint_trajectory.joint_names[i]);
        }
        if (arms_trajectory.joint_trajectory.joint_names[i].compare(0,arm2_prefix.size(),arm2_prefix)==0){
            arm2_trajectory.joint_trajectory.joint_names.push_back(arms_trajectory.joint_trajectory.joint_names[i]);
        }
    }

    // points
    arm1_trajectory.joint_trajectory.points.resize(arms_trajectory.joint_trajectory.points.size());
    arm2_trajectory.joint_trajectory.points.resize(arms_trajectory.joint_trajectory.points.size());
    for (unsigned int i = 0; i < arms_trajectory.joint_trajectory.points.size(); i++){
        for (unsigned int j = 0; j < arms_trajectory.joint_trajectory.joint_names.size(); j++){

            if (arms_trajectory.joint_trajectory.joint_names[j].compare(0,arm1_prefix.size(),arm1_prefix)==0){
                /*arm1_trajectory.joint_trajectory.points[i].accelerations.push_back(
                        arms_trajectory.joint_trajectory.points[i].accelerations[j]);
                arm1_trajectory.joint_trajectory.points[i].effort.push_back(
                        arms_trajectory.joint_trajectory.points[i].effort[j]);*/
                arm1_trajectory.joint_trajectory.points[i].positions.push_back(
                        arms_trajectory.joint_trajectory.points[i].positions[j]);
                arm1_trajectory.joint_trajectory.points[i].velocities.push_back(
                        arms_trajectory.joint_trajectory.points[i].velocities[j]);
            }
            else if (arms_trajectory.joint_trajectory.joint_names[j].compare(0,arm2_prefix.size(),arm2_prefix)==0){
                /*arm2_trajectory.joint_trajectory.points[i].accelerations.push_back(
                        arms_trajectory.joint_trajectory.points[i].accelerations[j]);
                arm2_trajectory.joint_trajectory.points[i].effort.push_back(
                        arms_trajectory.joint_trajectory.points[i].effort[j]);*/
                arm2_trajectory.joint_trajectory.points[i].positions.push_back(
                        arms_trajectory.joint_trajectory.points[i].positions[j]);
                arm2_trajectory.joint_trajectory.points[i].velocities.push_back(
                        arms_trajectory.joint_trajectory.points[i].velocities[j]);
            }
        }
        arm1_trajectory.joint_trajectory.points[i].time_from_start =
                arms_trajectory.joint_trajectory.points[i].time_from_start;
        arm2_trajectory.joint_trajectory.points[i].time_from_start =
                arms_trajectory.joint_trajectory.points[i].time_from_start;
    }
    return true;
}
// Check the time sequence in the trajectory points
void TrajectoryProcessor::clean(moveit_msgs::RobotTrajectory &trajectory) {
    for (unsigned int i = 1; i < trajectory.joint_trajectory.points.size(); i++){
        if (trajectory.joint_trajectory.points[i-1].time_from_start >= trajectory.joint_trajectory.points[i].time_from_start){
            ROS_WARN("Detected that trajectory is not increasing in time. Erased false entry.");
            trajectory.joint_trajectory.points.erase(trajectory.joint_trajectory.points.begin()+i-1);
        }
    }
}

void TrajectoryProcessor::computeTimeFromStart(moveit_msgs::RobotTrajectory& trajectory, double step_t){
    for (unsigned int i=0; i < trajectory.joint_trajectory.points.size(); i++){
        trajectory.joint_trajectory.points[i].time_from_start.fromSec(i*step_t);
    }
}

void TrajectoryProcessor::scaleTrajectorySpeed(moveit_msgs::RobotTrajectory& trajectory, double scale){
    for (unsigned int i = 0; i < trajectory.joint_trajectory.points.size(); i++){
        trajectory.joint_trajectory.points[i].time_from_start.fromSec(trajectory.joint_trajectory.points[i].time_from_start.toSec()/scale);
        for (unsigned int j = 0; j < trajectory.joint_trajectory.joint_names.size(); j++){
            trajectory.joint_trajectory.points[i].velocities[j] = trajectory.joint_trajectory.points[i].velocities[j] * scale;
            trajectory.joint_trajectory.points[i].accelerations[j] = trajectory.joint_trajectory.points[i].accelerations[j] * scale;
        }
    }
}

bool TrajectoryProcessor::computeVelocities(moveit_msgs::RobotTrajectory& trajectory, moveit::planning_interface::MoveGroup& moveGroup){
    dual_arm_toolbox::TrajectoryProcessor::computeTimeFromStart(trajectory, 0.4);
    robot_trajectory::RobotTrajectory rt(moveGroup.getCurrentState()->getRobotModel(), "arms");
   
    rt.setRobotTrajectoryMsg(*moveGroup.getCurrentState(), trajectory);// get a RobotTrajectory from trajectory
     /* This class modifies the timestamps of a trajectory to respect velocity and acceleration contraints */
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    if (!iptp.computeTimeStamps(rt)){
        ROS_WARN("computed time stamp FAILED.");
        return false;
    }
    rt.getRobotTrajectoryMsg(trajectory);// Get RobotTrajectory_msg from RobotTrajectory
    return true;
}

void TrajectoryProcessor::visualizePlan(moveit::planning_interface::MoveGroup::Plan& plan, unsigned int sec){
    ros::NodeHandle nh;
    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = plan.start_state_;
    display_trajectory.trajectory.push_back(plan.trajectory_);
    ROS_INFO("Visualizing plan and waiting for %i seconds", sec);
    display_publisher.publish(display_trajectory);
    sleep(sec);
}

void TrajectoryProcessor::publishPlanTrajectory(moveit::planning_interface::MoveGroup::Plan& plan, unsigned int sec){
    ros::NodeHandle nh;
    ros::Publisher execTrajectoryPub_ = nh.advertise<moveit_msgs::RobotTrajectory>("/execute_my_trajectory", 1, true);
    moveit_msgs::RobotTrajectory trajectory_ = plan.trajectory_;
    // ROS_INFO("Publishing plan and waiting for %i seconds", sec);
    execTrajectoryPub_.publish(trajectory_);
    sleep(sec);
    // ROS_INFO("Header time  %f ", trajectory_.joint_trajectory.header.stamp.toSec());
        // for (unsigned int i = 0; i < trajectory_.joint_trajectory.points.size(); i++){
        //     ROS_INFO("Listening Points %d  %f ", i, trajectory_.joint_trajectory.header.stamp.toSec()+trajectory_.joint_trajectory.points[i].time_from_start.toSec());
        //     for (unsigned int a = 0; a < trajectory_.joint_trajectory.points[i].positions.size(); a++){
        //         /ROS_INFO("%s:\tpos %f\tvel %f", 
        //         trajectory_.joint_trajectory.joint_names[a].c_str(), 
        //         trajectory_.joint_trajectory.points[i].positions[a]*(180/3.14159),
        //         trajectory_.joint_trajectory.points[i].velocities[a]*(180/3.14159));
        //     }
        // }
}