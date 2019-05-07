// rosrun dual_arm_robot_applications left_force_hold_exp

#include <moveit/move_group_interface/move_group_interface.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/SwitchControllerRequest.h>
#include <controller_manager_msgs/SwitchControllerResponse.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

// Controller Interface
#include <moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

#include "ur_logging/UrLogger.h"

// KDL
#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include "dual_arm_demonstrator_iml/DualArmRobot.h"
#include <dual_arm_demonstrator_iml/SceneManager.h>
#include "dual_arm_demonstrator_iml/FTSensorSubscriber.h"
int main(int argc, char **argv)
{
    // ROS Setup
    ros::init(argc, argv, "ur_const_vel_publisher");
    ros::NodeHandle nh;
    ros::Publisher c_joint_speed_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/left/ur_driver/joint_speed", 1);
    ros::Publisher ur_script_pub = nh.advertise<std_msgs::String>("/left/ur_driver/URScript", 1);
    ros::AsyncSpinner asyncSpinner(2);
    asyncSpinner.start();

    // create ur_logger. Use this namespace
    std::vector<std::string> ur_namespaces;
    ur_namespaces.push_back("left");
    UR_Logger ur_logger(nh, ur_namespaces);
    // start logging
    ur_logger.start(50);
    FTSensorSubscriber FTsubscriber(nh, ur_namespaces[0]);
    // Controller Interface
    std::string left_controller_ = "left/vel_based_pos_traj_controller";
    // MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface left_("left_manipulator");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    left_.setPlanningTime(30);

    std_msgs::String temp;
    std::string cmd_str;
    //        std::string force_mode="force_mode( tool_pose(), [0, 0, 1, 0, 0, 0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2, [0.05, 0.05, 0.05, 0.17, 0.17, 0.17])\n";
    //        std::string free_drive_mode = "\tfreedrive_mode()\n";
    // while(ros::ok()){
    //     bool last_io_button = true;
    //         ros::spinOnce();
    //         if(!last_io_button && io_button_){
    //             cmd_str = "def myProg():\n";
    //             cmd_str += "\twhile (True):\n";
    //             cmd_str += "\t\tfreedrive_mode()\n";
    //             cmd_str +="\t\tsync()\n";
    //             cmd_str += "\tend\n";
    //             cmd_str +="end\n";
    //             temp.data = cmd_str;
    //             pub_free_drive_.publish(temp);
    //         }
    //         if(last_io_button && !io_button_){
    //             cmd_str = "def myProg():\n";
    //             cmd_str += "\twhile (True):\n";
    //             cmd_str += "\t\tend_freedrive_mode()\n";
    //             cmd_str +="\t\tsleep(0.5)\n";
    //             cmd_str += "\tend\n";
    //             cmd_str +="end\n";
    //             temp.data = cmd_str;
    //             pub_free_drive_.publish(temp);
    //         }

    ROS_WARN("robot is moving without collision checking. BE CAREFUL!");
    ROS_INFO("waiting 10 Seconds. Press Ctrl-C if Robot is in the wrong start position");
    ROS_INFO("Reference Frame: %s", left_.getPlanningFrame().c_str());
    ROS_INFO("EndEffectorLink: %s", left_.getEndEffectorLink().c_str());
    geometry_msgs::PoseStamped left_current_pose_ = left_.getCurrentPose(left_.getEndEffectorLink());
    ROS_INFO("\nleft_current_pose_ frame_id: %s, end_effector: %s, x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f\n",
             left_current_pose_.header.frame_id.c_str(), left_.getEndEffectorLink().c_str(), left_current_pose_.pose.position.x, left_current_pose_.pose.position.y, left_current_pose_.pose.position.z, left_current_pose_.pose.orientation.x, left_current_pose_.pose.orientation.y, left_current_pose_.pose.orientation.z, left_current_pose_.pose.orientation.w);

    std::vector<std::string> leftJointNames = left_.getActiveJoints();
    std::vector<double> leftJointValues = left_.getCurrentJointValues();
    for (std::size_t i = 0; i < leftJointNames.size(); i++)
    {
        ROS_INFO("Joint %s: %f", leftJointNames[i].c_str(), leftJointValues[i]);
    }
    ros::Duration(10).sleep();
    // ::::::: Run Experiments :::::::
    // variables
    double velocity = 0.005;
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

    geometry_msgs::PoseStamped current_pose = left_.getCurrentPose();
    double radius = sqrt(current_pose.pose.position.x * current_pose.pose.position.x + current_pose.pose.position.y * current_pose.pose.position.y);

    double omega = velocity / radius;

    std::cout << "RADIUS: " << radius << "  omega: " << omega << std::endl;
    //double moving_time = moving_distance/velocity; //move 3cm, obstacle is around 1-2 cm away.

    // Output Message Speed = const
    trajectory_msgs::JointTrajectory joint_traj; //containing speed command
    trajectory_msgs::JointTrajectoryPoint traj_point;
    traj_point.velocities.assign(6, 0);
    traj_point.velocities[5] = 0.1;
    // joint_traj.points.push_back(traj_point);

    geometry_msgs::TwistStamped tool_velocity;
    geometry_msgs::Twist t;
    t.linear.x = 0;
    t.linear.y = 0.05;
    t.linear.z = 0;

    t.angular.x = 0;
    t.angular.y = 0;
    t.angular.z = 0;

    tool_velocity.twist = t;

    ROS_INFO("Publishing velocity commands to left_ at 100Hz");

    // publish messages
    ros::Rate loop_rate(100); // velocity-message publish rate

    Stopwatch stopwatch;

    ROS_INFO("trying to hold 10N");
    for (int i = 0; i < joint_traj.joint_names.size(); ++i)
    {
        ROS_INFO("Joint name  %s", joint_traj.joint_names[i].c_str());
    }
    int loop = 10;
    while (ros::ok() && loop > 0)
    {
        double res_force = sqrt(FTsubscriber.last_wrench_msg_.wrench.force.x * FTsubscriber.last_wrench_msg_.wrench.force.x + FTsubscriber.last_wrench_msg_.wrench.force.y * FTsubscriber.last_wrench_msg_.wrench.force.y);

        if (loop % 2 == 0)
        {
            traj_point.velocities[5] = 1;
            joint_traj.points.push_back(traj_point);
            joint_traj.header.stamp = ros::Time::now();
            c_joint_speed_pub.publish(joint_traj);
            sleep(2);
        }
        else
        {

            traj_point.velocities[5] = -1;
            joint_traj.points.push_back(traj_point);
            joint_traj.header.stamp = ros::Time::now();
            c_joint_speed_pub.publish(joint_traj);
            sleep(2);
        }
        loop--;
        /*
        if (res_force < 10){
            // Removes the last element in the vector, effectively reducing the container size by one.
            // joint_traj.points.pop_back();
            // traj_point.velocities[2] = 0.1;
            // joint_traj.points.push_back(traj_point);

            t.linear.y = 0.05;
            ROS_INFO("I heard: Force [%f]  FX[%f] FY[%f] FZ[%f]", res_force, FTsubscriber.last_wrench_msg_.wrench.force.x, FTsubscriber.last_wrench_msg_.wrench.force.y, FTsubscriber.last_wrench_msg_.wrench.force.z);

        }
        else if (res_force  < 30){
            // joint_traj.points.pop_back();
            // traj_point.velocities[2] = 0.0;
            // joint_traj.points.push_back(traj_point);
            t.linear.y = -0.05;
        }
        else{
            // joint_traj.points.pop_back();
            // traj_point.velocities[0] = -0.1;
            // joint_traj.points.push_back(traj_point);

            t.linear.y = 0;
        }
        */
        // joint_traj.header.stamp = ros::Time::now();
        // c_joint_speed_pub.publish(joint_traj);

        // tool_velocity.header.stamp = ros::Time::now();
        // left_tool_vel_pub_.publish(tool_velocity);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("moving back");
    stopwatch.restart();

    // while (ros::ok() && (stopwatch.elapsed().toSec()<10))
    // {
    //     joint_traj.header.stamp = ros::Time::now();
    //     c_joint_speed_pub.publish(joint_traj);

    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    ROS_INFO("Stopped publishing");

    // restart controller
    srv_req.request.BEST_EFFORT;
    srv_req.request.start_controllers.push_back("vel_based_pos_traj_controller");
    success = left_srv_switch_controller.call(srv_req);
    ROS_INFO("Starting controller %s", success ? "SUCCEDED" : "FAILED");
    srv_req.request.start_controllers.clear();

    // ROS_INFO("moveing to start");
    // left_.plan(plan);
    // left_.execute(plan);

    // stop logging
    ur_logger.stop();
    ROS_INFO("finished. shutting down.");

    ros::shutdown();
    return 0;
}
