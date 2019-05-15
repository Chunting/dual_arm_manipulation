//
// Created by Chunting  on 17.10.16.
//

#include "dual_arm_toolbox/TrajectoryProcessor.h"

using namespace dual_arm_toolbox;
// fuses two trajectories into one; both should be of same length
bool TrajectoryProcessor::fuse(moveit_msgs::RobotTrajectory &arms_trajectory,
                               moveit_msgs::RobotTrajectory arm1_trajectory,
                               moveit_msgs::RobotTrajectory arm2_trajectory) {
    int i1 = arm1_trajectory.joint_trajectory.points.size();
    int i2 = arm2_trajectory.joint_trajectory.points.size();
    int num = std::min(i1,i2);
    ROS_INFO("Start to fuse trajectory i1=%d, i2=%d, num=%d", i1, i2, num);
    if(i1==0 && i2==0){
        ROS_ERROR(" Can not fuse empty trajectory");
        return false;
    }
    if(i1==0){
        arms_trajectory = arm2_trajectory;
        return true;
    }
    if(i2==0){
        arms_trajectory = arm1_trajectory;
        return true;
    }
    arms_trajectory.joint_trajectory.points.resize(num);
    if(i1<=i2){
        /* If arm1_trajectory has fewer points, remove the first points of arm2_trajectory,
           then the two arms have the same number of points to run*/
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
        for (unsigned int i=0; i < arm1_trajectory.joint_trajectory.joint_names.size(); i++)
            arms_trajectory.joint_trajectory.joint_names.push_back(arm1_trajectory.joint_trajectory.joint_names[i]);
        for (unsigned int i=0; i < arm2_trajectory.joint_trajectory.joint_names.size(); i++)
            arms_trajectory.joint_trajectory.joint_names.push_back(arm2_trajectory.joint_trajectory.joint_names[i]);
        for (unsigned int i=0; i < num; i++){
            int offset = i1-num;
            arms_trajectory.joint_trajectory.points[i].time_from_start = arm2_trajectory.joint_trajectory.points[i].time_from_start;
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
    if(arms_trajectory.joint_trajectory.joint_names.empty() || arms_trajectory.joint_trajectory.points.empty()){
        ROS_ERROR("Empty arms trajectory!");
        return false;
    }

    // joint names
    for (unsigned int i = 0; i < arms_trajectory.joint_trajectory.joint_names.size(); i++){
        if (arms_trajectory.joint_trajectory.joint_names[i].compare(0,arm1_prefix.size(),arm1_prefix)==0){
            arm1_trajectory.joint_trajectory.joint_names.push_back(arms_trajectory.joint_trajectory.joint_names[i]);
        }
        if (arms_trajectory.joint_trajectory.joint_names[i].compare(0,arm2_prefix.size(),arm2_prefix)==0){
            arm2_trajectory.joint_trajectory.joint_names.push_back(arms_trajectory.joint_trajectory.joint_names[i]);
        }
    }
    // if(arm1_trajectory.joint_trajectory.joint_names.size() == 0) {
    //     arm2_trajectory = arms_trajectory;
    //     return true;
    // }
    // if(arm2_trajectory.joint_trajectory.joint_names.size() == 0) {
    //     arm1_trajectory = arms_trajectory;
    //     return true;
    // }

     // header
    arm1_trajectory.joint_trajectory.header = arms_trajectory.joint_trajectory.header;
    arm2_trajectory.joint_trajectory.header = arms_trajectory.joint_trajectory.header;

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
            ROS_WARN("Detected that trajectory is not increasing in time. Erased false entry, %d", i);
            trajectory.joint_trajectory.points.erase(trajectory.joint_trajectory.points.begin()+i-1);
        }
    }
}

// Set timestamp
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

bool TrajectoryProcessor::computeVelocities(moveit_msgs::RobotTrajectory& trajectory, moveit::planning_interface::MoveGroupInterface& moveGroup){
    dual_arm_toolbox::TrajectoryProcessor::computeTimeFromStart(trajectory, 0.4); // 400ms
    // Maintain a sequence of waypoints and the time durations between these waypoints.
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

void TrajectoryProcessor::visualizePlan(moveit::planning_interface::MoveGroupInterface::Plan& plan, unsigned int sec){
    ros::NodeHandle nh;
    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = plan.start_state_;
    display_trajectory.trajectory.push_back(plan.trajectory_);
    ROS_INFO("Visualizing plan and waiting for %i seconds", sec);
    display_publisher.publish(display_trajectory);
    sleep(sec);
}

void TrajectoryProcessor::publishPlanTrajectory(moveit::planning_interface::MoveGroupInterface::Plan& plan, unsigned int sec){
    ros::NodeHandle nh;
    ros::Publisher execTrajectoryPub_ = nh.advertise<moveit_msgs::RobotTrajectory>("/robot_traj_cmd", 1, true);
    moveit_msgs::RobotTrajectory trajectory_ = plan.trajectory_;
    execTrajectoryPub_.publish(trajectory_);
    sleep(sec);
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
}
void TrajectoryProcessor::publishJointTrajectory(ros::NodeHandle &nh, std::string ur_namespace, moveit::planning_interface::MoveGroupInterface::Plan& plan){
    std::string joint_traj_topic = "/joint_traj_cmd";
    std::string joint_traj_point_topic = "/joint_traj_point_cmd";
    if(ur_namespace == "left"){
        joint_traj_topic = "left/joint_traj_cmd";
        joint_traj_point_topic = "left/joint_traj_point_cmd";
    } else if(ur_namespace == "right"){
        joint_traj_topic = "right/joint_traj_cmd";
        joint_traj_point_topic = "right/joint_traj_point_cmd";
    }
    
    ros::Publisher pub_joint_traj_cmd_ = nh.advertise<trajectory_msgs::JointTrajectory>(joint_traj_topic, 1);
    
    ros::Publisher pub_joint_traj_point_cmd_ = nh.advertise<trajectory_msgs::JointTrajectoryPoint>(joint_traj_point_topic, 1);
    trajectory_msgs::JointTrajectory joint_trajectory = plan.trajectory_.joint_trajectory;
    pub_joint_traj_cmd_.publish(joint_trajectory);
    ROS_INFO("Publish joint trajectory on topic %s", joint_traj_topic.c_str()); 
    ros::Rate loop_rate_(100);
    for(int i=0; i<joint_trajectory.points.size(); ++i){
        trajectory_msgs::JointTrajectoryPoint point = joint_trajectory.points[i];
        pub_joint_traj_point_cmd_.publish(joint_trajectory.points[i]);
        // for (unsigned int a = 0; a < point.positions.size(); a++){
        //     ROS_INFO("%s:\tpos %f\tvel %f", 
        //     joint_trajectory.joint_names[a].c_str(), 
        //     point.positions[a]*(180/3.14159),
        //     point.velocities[a]*(180/3.14159));
        // }
        // loop_rate_.sleep();
    }
}
void TrajectoryProcessor::PrintTrajectory(const moveit_msgs::RobotTrajectory &trajectory)
{
    for (unsigned int i = 0; i < trajectory.joint_trajectory.points.size(); i++)
    {
        ROS_INFO("Points %d", i);
        for (unsigned int a = 0; a < trajectory.joint_trajectory.points[i].positions.size(); a++)
        {
            ROS_INFO("%s: pos %f\t vel %f",
                     trajectory.joint_trajectory.joint_names[a].c_str(),
                     trajectory.joint_trajectory.points[i].positions[a],
                     trajectory.joint_trajectory.points[i].velocities[a]);
        }
    }
}
