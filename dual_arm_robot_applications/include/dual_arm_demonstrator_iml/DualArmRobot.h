//
// Created by Chunting on 27.12.16.
//

#ifndef PROJECT_DUALARMROBOT_H
#define PROJECT_DUALARMROBOT_H

// Robot status
//#define OFFLINE
#define PI 3.14159
// Dual Arm Toolbox
#include "dual_arm_toolbox/TrajectoryProcessor.h"
#include "dual_arm_toolbox/Transform.h"

// Controller Manager
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/SwitchControllerRequest.h>
#include <controller_manager_msgs/SwitchControllerResponse.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <controller_manager_msgs/SwitchController.h>
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

// Geometry
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>

// tf
#include <tf/tf.h>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// KDL
#include <kdl/frames_io.hpp>
#include <eigen_conversions/eigen_kdl.h>

// Controller Interface
#include <moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h>


namespace dual_arm_demonstrator_iml {
    class DualArmRobot {
    protected:
        ros::NodeHandle nh_;
        std::string left_controller_;
        std::string right_controller_;
        moveit_msgs::RobotState current_dual_arm_state_; // mounted with force sensor
        double ee_dist_;    // distance to object because of endeffector size
        KDL::Frame arms_offset_;    // offset between arms
        bool try_again_question();
        // PlanningSceneMonitorPtr is aka std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>
        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

        robot_model_loader::RobotModelLoader robotModelLoader;
        robot_model::RobotModelPtr kinematic_model;
        //planning_scene::PlanningScene planningScene;


        // Construct a RobotState that maintains the configuration of the robot.
        robot_state::RobotStatePtr kinematic_state;
        // setup JointModelGroup
        // Represents the robot model for a particular group.
        const robot_state::JointModelGroup* left_joint_model_group;
        const robot_state::JointModelGroup* right_joint_model_group;

    public:
        DualArmRobot(ros::NodeHandle &nh);
        // Get the current pose of end effector
        geometry_msgs::PoseStamped left_current_pose_;
        geometry_msgs::PoseStamped left_current_pose_temp_;

        geometry_msgs::PoseStamped right_current_pose_;
        geometry_msgs::PoseStamped right_current_pose_temp_;
        // The MoveGroup class can be easily setup using the group name
        moveit::planning_interface::MoveGroup left_;
        moveit::planning_interface::MoveGroup right_;
        moveit::planning_interface::MoveGroup arms_;

        std::vector<double> left_joint_values;
        std::vector<double> right_joint_values;

        // workaround for moveGroup method does not return attached objects correctly (issue)
        robot_state::RobotState getCurrentRobotState();
        moveit_msgs::RobotState getCurrentRobotStateMsg();

        // calculates a trajectory for both arms based on the trajectory of one arm
        bool adaptTrajectory(moveit_msgs::RobotTrajectory left_trajectory, KDL::Frame offset,
                             moveit_msgs::RobotTrajectory &both_arms_trajectory, double jump_threshold = 0.4);

        // returns the Offset-Vector between both end effectors
        KDL::Frame getCurrentOffset();

        // executes a pick: Moves both arms to the object, grasps it, moves up
        bool pickBox(std::string object_id, geometry_msgs::Vector3Stamped lift_direction);

        // pace Methods
        bool placeBox(std::string object_id, geometry_msgs::PoseStamped left_place_pose,
                      geometry_msgs::Vector3 close_direction);

        bool pushPlaceBox(std::string object_id, geometry_msgs::PoseStamped box_pose, geometry_msgs::Vector3 direction);

        bool moveObject(std::string object_id, geometry_msgs::PoseStamped left_pose, double scale=0.2);
        bool planMoveObject(std::string object_id, geometry_msgs::PoseStamped left_pose, double scale=0.2); // Only plan an visualize without executing. For use in validation.

        // enable/disable collision check between robot arms
        void allowedArmCollision(bool enable, std::string left_attachedObject);

        bool place(geometry_msgs::Pose pose_object);

        bool switch_controller(std::string stop_name, std::string start_name, std::string ur_namespace);

        bool graspMove(double distance, bool avoid_collisions = true, bool use_left=true, bool use_right=true); //both arms will be moved closer together

        bool linearMove(geometry_msgs::Vector3Stamped direction, bool avoid_collisions = true, bool use_left=true, bool use_right=true);

        bool linearMoveParallel(geometry_msgs::Vector3Stamped direction, std::string object_id, double traj_scale=1, bool avoid_collisions = true);

        bool execute(moveit::planning_interface::MoveGroup::Plan plan);

        bool moveHome();
        bool moveGraspPosition();
        // Planning to a joint-space goal
        std::vector<double> getJointAngles(std::string groupName);

        moveit_msgs::RobotState getPositionIK(std::string& groupName, moveit_msgs::RobotState& seed_robot_state, geometry_msgs::PoseStamped poseIK);
        std::vector<double>  getPositionIK(const robot_state::JointModelGroup* joint_model_group, const geometry_msgs::Pose &pose);

        geometry_msgs::PoseStamped getPositionFK(std::string& endEffectorLink, moveit_msgs::RobotState& seed_robot_state);
        // Eigen::Affine3d& getPositionFK(const robot_state::JointModelGroup* joint_model_group, std::string& endEffectorLink);

        Eigen::MatrixXd  getJacobian(const robot_state::JointModelGroup* joint_model_group, Eigen::Vector3d &reference_point_position );

        void PrintPose(geometry_msgs::Pose &pose);
        double radianToDegree(double radian){
            
            return (radian*(180/PI));
        }
    };
}//ns
#endif //PROJECT_DUALARMROBOT_H
