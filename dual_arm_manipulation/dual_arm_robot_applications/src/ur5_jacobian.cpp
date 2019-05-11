#include <ros/ros.h>
#include <std_msgs/Float64.h>
// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
// controller manager
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/SwitchControllerRequest.h>
#include <controller_manager_msgs/SwitchControllerResponse.h>
// Controller Interface
#include <moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h>
// Trajectory process
#include "dual_arm_toolbox/TrajectoryProcessor.h"
#include "dual_arm_toolbox/Transform.h"

std::vector<double> joint_speeds;
std::vector<double> joint_values;
bool data_come = false;
std_msgs::Header header;

void chatterCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    if (msg->velocity.size() == 6)
    {
        //ROS_INFO(" I heard velocities: %lf, %lf, %lf, %lf, %lf, %lf", msg->velocity[0], msg->velocity[1], msg->velocity[2], msg->velocity[3], msg->velocity[4], msg->velocity[5]);
        //ROS_INFO(" I heard positions: %lf, %lf, %lf, %lf, %lf, %lf", msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5]);
        joint_speeds[0] = msg->velocity[0];
        joint_speeds[1] = msg->velocity[1];
        joint_speeds[2] = msg->velocity[2];
        joint_speeds[3] = msg->velocity[3];
        joint_speeds[4] = msg->velocity[4];
        joint_speeds[5] = msg->velocity[5];
        joint_values[0] = msg->position[0];
        joint_values[1] = msg->position[1];
        joint_values[2] = msg->position[2];
        joint_values[3] = msg->position[3];
        joint_values[4] = msg->position[4];
        joint_values[5] = msg->position[5];
        header = msg->header;
        data_come = true;
    }
}
int main(int argc, char **argv)
{
    joint_speeds.push_back(0);
    joint_speeds.push_back(0);
    joint_speeds.push_back(0);
    joint_speeds.push_back(0);
    joint_speeds.push_back(0);
    joint_speeds.push_back(0);
    joint_values.push_back(0);
    joint_values.push_back(0);
    joint_values.push_back(0);
    joint_values.push_back(0);
    joint_values.push_back(0);
    joint_values.push_back(0);
    ros::init(argc, argv, "left_arm_kinematics");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(100);
  
   
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // stop controller
    ros::ServiceClient left_srv_switch_controller = nh.serviceClient<controller_manager_msgs::SwitchController>("/left/controller_manager/switch_controller");
    controller_manager_msgs::SwitchController srv_req;
    srv_req.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
    srv_req.request.stop_controllers.push_back("/left/vel_based_pos_traj_controller");
    bool success = left_srv_switch_controller.call(srv_req);
    ROS_INFO("Stopping controller %s", success ? "SUCCEDED" : "FAILED");
    if (!success)
        return 0;
    srv_req.request.stop_controllers.clear();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 

    ros::Subscriber sub = nh.subscribe("/joint_states", 1, chatterCallback);
    ros::Publisher m_pub_tcp_speed = nh.advertise<geometry_msgs::TwistStamped>("/left/m_tool_velocity", 1);
    ros::Publisher c_pub_tcp_speed = nh.advertise<geometry_msgs::TwistStamped>("/left/c_tool_velocity", 1);
    ros::Publisher c_joint_speed_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/left/ur_driver/joint_speed", 1000);
    Eigen::MatrixXd joint_velocities(6, 1);
    Eigen::MatrixXd tcp_velocities(6, 1);
    geometry_msgs::TwistStamped tcp_msg;
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd jacobian_inverse;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("left_manipulator");
    ROS_INFO("Model frame: %s\tLast Link %s", kinematic_model->getModelFrame().c_str(), joint_model_group->getLinkModelNames().back().c_str());

    // MoveIt! Setup
    // The MoveGroupInterface class can be easily setup using the group name
    planning_scene::PlanningScene planningScene(kinematic_model);
    moveit::planning_interface::MoveGroupInterface left_("left_manipulator");
    // Spceciy the maximum amount of time to use when planning
    left_.setPlanningTime(5);
    // Set the number of times the motion plan is to be computed, the default value is 1
    left_.setNumPlanningAttempts(10);
    // setup planner
    // Specify a planner to be used for further planning
    left_.setPlannerId("RRTConnectkConfigDefault");
    // Allow replanning
    left_.allowReplanning(true);
    // Controller Interface
    std::string left_controller_ = "left/vel_based_pos_traj_controller";

    // const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
    // Get Joint Values
    // ^^^^^^^^^^^^^^^^
    // We can retreive the current set of joint values stored in the state for the right arm.
    const std::vector<std::string> &joint_names = left_.getActiveJoints();
    std::vector<double> joint_values = left_.getCurrentJointValues();
    // kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); i++)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    trajectory_msgs::JointTrajectory trj;
    trajectory_msgs::JointTrajectoryPoint trjp;
    trjp.velocities.push_back(0.0);
    trjp.velocities.push_back(0.0);
    trjp.velocities.push_back(0.0);
    trjp.velocities.push_back(0.0);
    trjp.velocities.push_back(0.0);
    trjp.velocities.push_back(0.0);
    trj.points.push_back(trjp);
    float tcp_velocity = 0.3;
    int direction = 1;
    int counter = 0;
    ros::Time be = ros::Time::now();
    while (ros::ok())
    {
        if (data_come)
        {
            ros::Time k = ros::Time::now();
            data_come = false;

            ROS_INFO("measured joint_speed: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
                     joint_speeds[0], joint_speeds[1], joint_speeds[2], joint_speeds[3], joint_speeds[4], joint_speeds[5]);
            // ROS_INFO("measured joint_values: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
            //          joint_values[0], joint_values[1], joint_values[2], joint_values[3], joint_values[4], joint_values[5]);
            kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

            kinematic_state->getJacobian(joint_model_group,
                                         kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                         reference_point_position, // The reference point position (with respect to the link specified in link_name)
                                         jacobian);
            joint_velocities(0, 0) = joint_speeds[0];
            joint_velocities(1, 0) = joint_speeds[1];
            joint_velocities(2, 0) = joint_speeds[2];
            joint_velocities(3, 0) = joint_speeds[3];
            joint_velocities(4, 0) = joint_speeds[4];
            joint_velocities(5, 0) = joint_speeds[5];
            

            tcp_velocities = jacobian * joint_velocities;
			ROS_INFO_STREAM("TCP velocities: \n" << tcp_velocities);
            tcp_msg.header = header;
			tcp_msg.twist.linear.x = tcp_velocities(0,0);
			tcp_msg.twist.linear.y = tcp_velocities(1,0);
			tcp_msg.twist.linear.z = tcp_velocities(2,0);
			tcp_msg.twist.angular.x = tcp_velocities(3,0);
			tcp_msg.twist.angular.y = tcp_velocities(4,0);
			tcp_msg.twist.angular.z = tcp_velocities(5,0);
			m_pub_tcp_speed.publish(tcp_msg);
            jacobian_inverse = jacobian.inverse();

            // if (counter % 3 == 0)
            // {
                tcp_velocities(0, 0) = 0;// tcp_velocity;// * direction;
                tcp_velocities(1, 0) = 0.1;
                tcp_velocities(2, 0) = 0;
                tcp_velocities(3, 0) = 0;
                tcp_velocities(4, 0) = 0;
                tcp_velocities(5, 0) = 0;
            // }
            // else if (counter % 3 == 1)
            // {
            //     tcp_velocities(0, 0) = 0;
            //     tcp_velocities(1, 0) = tcp_velocity * direction;
            //     tcp_velocities(2, 0) = 0;
            //     tcp_velocities(3, 0) = 0;
            //     tcp_velocities(4, 0) = 0;
            //     tcp_velocities(5, 0) = 0;
            // }
            // else
            // {
            //     tcp_velocities(0, 0) = 0;
            //     tcp_velocities(1, 0) = 0;
            //     tcp_velocities(2, 0) = tcp_velocity * direction;
            //     tcp_velocities(3, 0) = 0;
            //     tcp_velocities(4, 0) = 0;
            //     tcp_velocities(5, 0) = 0;
            // }
            // ROS_INFO_STREAM("TCP velocities: \n"
            //                 << tcp_velocities);

            // ROS_INFO_STREAM("Jacobian: \n"
            //                 << jacobian);
            // ROS_INFO_STREAM("Inverse of Jacobian: \n"
            //                 << jacobian_inverse);
            joint_velocities = jacobian_inverse * tcp_velocities;
            std_msgs::Float64 vel;
            ros::Duration du = k - be;
		    vel.data = 0;
            vel.data = 0.15*sin(du.toSec());
		    trj.points[0].velocities[5] = vel.data;

            // trj.points[0].velocities[0] = joint_velocities(0, 0);
            // trj.points[0].velocities[1] = joint_velocities(1, 0);
            // trj.points[0].velocities[2] = joint_velocities(2, 0);
            // trj.points[0].velocities[3] = joint_velocities(3, 0);
            // trj.points[0].velocities[4] = joint_velocities(4, 0);
            // trj.points[0].velocities[5] = joint_velocities(5, 0);
            
            // trj.points[0].velocities[0] = 0;
            // trj.points[0].velocities[1] = 0;
            // trj.points[0].velocities[2] = 0;
            // trj.points[0].velocities[3] = 0;
            // trj.points[0].velocities[4] = 0;
            // trj.points[0].velocities[5] = 0.1*direction;//joint_velocities(5, 0);
            trj.header.stamp = ros::Time::now();
            c_joint_speed_pub.publish(trj);
            // ROS_INFO_STREAM("Joint velocities: \n"
            //                 << joint_velocities);

            tcp_msg.header = header;
            tcp_msg.twist.linear.x = tcp_velocities(0, 0);
            tcp_msg.twist.linear.y = tcp_velocities(1, 0);
            tcp_msg.twist.linear.z = tcp_velocities(2, 0);
            tcp_msg.twist.angular.x = tcp_velocities(3, 0);
            tcp_msg.twist.angular.y = tcp_velocities(4, 0);
            tcp_msg.twist.angular.z = tcp_velocities(5, 0);
            c_pub_tcp_speed.publish(tcp_msg);
            direction = -direction;
            counter++;
            ros::spinOnce();
            loop_rate.sleep();
            // sleep(2);
        }
    }

    // trj.points[0].velocities[0] = 0;
    // trj.header.stamp = ros::Time::now();
    // c_joint_speed_pub.publish(trj);
    // END_TUTORIAL
    
    sleep(1);
    ros::shutdown();
    return 0;
}