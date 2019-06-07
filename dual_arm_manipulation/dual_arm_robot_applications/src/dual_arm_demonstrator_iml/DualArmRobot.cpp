//
// Created by Chunting  on 20.10.16.
//

#include <dual_arm_demonstrator_iml/SceneManager.h>
#include "dual_arm_demonstrator_iml/DualArmRobot.h"

using namespace dual_arm_demonstrator_iml;

DualArmRobot::DualArmRobot(ros::NodeHandle &nh) : left_("left_manipulator"),
                                                  right_("right_manipulator"),
                                                  arms_("arms"),
                                                  nh_(nh)
{

    /* Setup Kinematic Model */
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // BEGIN_TUTORIAL
    // Start
    // ^^^^^
    // Setting up to start using the RobotModel class is very easy. In
    // general, you will find that most higher-level components will
    // return a shared pointer to the RobotModel. You should always use
    // that when possible. In this example, we will start with such a
    // shared pointer and discuss only the basic API. You can have a
    // look at the actual code API for these classes to get more
    // information about how to use more features provided by these
    // classes.
    //
    // We will start by instantiating a
    // `RobotModelLoader`_
    // object, which will look up
    // the robot description on the ROS parameter server and construct a
    // :moveit_core:`RobotModel` for us to use.
    //
    // .. _RobotModelLoader: http://docs.ros.org/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
    robotModelLoader = robot_model_loader::RobotModelLoader("robot_description");
    kinematic_modelPtr = robotModelLoader.getModel();
    ROS_INFO("MoveIt Model Frame: %s \n", kinematic_modelPtr->getModelFrame().c_str());
    // Using the :moveit_core:`RobotModel`, we can construct a
    // :moveit_core:`RobotState` that maintains the configuration
    // of the robot.
    kinematic_statePtr = std::make_shared<moveit::core::RobotState>(kinematic_modelPtr);
    planningScenePtr = std::make_shared<planning_scene::PlanningScene>(kinematic_modelPtr);
    /* Set all joints to their default positions.
       The default position is 0, or if that is not within bounds then half way between min and max bound. */
    // kinematic_statePtr->setToDefaultValues();
    left_joint_model_group = kinematic_modelPtr->getJointModelGroup("left_manipulator");
    right_joint_model_group = kinematic_modelPtr->getJointModelGroup("right_manipulator");
#ifdef NO_CHANCE_THAT_THIS_SYMBOL_WILL_EVER_EXIST
    const kinematics::KinematicsBaseConstPtr& right_solver = right_joint_model_group->getSolverInstance();
    const kinematics::KinematicsBaseConstPtr& left_solver = left_joint_model_group->getSolverInstance();
    ROS_INFO("Kinematics solver %s for joint group %s:  ",
                      typeid(*right_solver).name(), right_joint_model_group->getName().c_str());
    ROS_INFO("Kinematics solver %s for joint group %s. ",
                      typeid(*left_solver).name(), left_joint_model_group->getName().c_str());
#endif

    // MoveIt! Setup
    //left_.setPlanningTime(40);
    // Spceciy the maximum amount of time to use when planning
    left_.setPlanningTime(5);
    // Set the number of times the motion plan is to be computed, the default value is 1
    left_.setNumPlanningAttempts(10);

    right_.setPlanningTime(5);
    right_.setNumPlanningAttempts(10);

    // setup planner
    // Specify a planner to be used for further planning
    left_.setPlannerId("RRTConnectkConfigDefault");
    right_.setPlannerId("RRTConnectkConfigDefault");
    // Allow replanning
    left_.allowReplanning(true);
    right_.allowReplanning(true);
    left_.setStartStateToCurrentState();
    right_.setStartStateToCurrentState();
    // Controller Interface
    // It has to be consistent with group ns and names in controllers.yaml
    left_controller_ = "left/vel_based_pos_traj_controller";
    right_controller_ = "right/vel_based_pos_traj_controller";
    pose_publish_thread_ = new std::thread(std::bind(&DualArmRobot::publishOffsetPointState, this));
    // planning scene monitor
    // Subscribes to the topic planning_scene
    planning_scene_monitorPtr_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

    // initialize left specific variables
    // left_.getEndEffectorLink() get current end-effector link
    // frame_id:  /world
    left_current_pose_ = left_.getCurrentPose(left_.getEndEffectorLink());
    right_current_pose_ = right_.getCurrentPose(right_.getEndEffectorLink());
    current_dual_arm_robotstate_msg_ = getCurrentRobotStateMsg();
    // Joint Limits
    // ^^^^^^^^^^^^
    // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
    setConstraints();
    kinematic_statePtr->enforceBounds();

    
#ifdef NO_CHANCE_THAT_THIS_SYMBOL_WILL_EVER_EXIST
    ROS_INFO("\nleft_current_pose_ frame_id: %s, end_effector: %s\n x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f\n",
             left_current_pose_.header.frame_id.c_str(), left_.getEndEffectorLink().c_str(),
             left_current_pose_.pose.position.x, left_current_pose_.pose.position.y, left_current_pose_.pose.position.z,
             left_current_pose_.pose.orientation.x, left_current_pose_.pose.orientation.y, left_current_pose_.pose.orientation.z, left_current_pose_.pose.orientation.w);
    ROS_INFO("\nright_current_pose_ frame_id: %s,  end_effector: %s\n x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f\n",
             right_current_pose_.header.frame_id.c_str(), right_.getEndEffectorLink().c_str(), right_current_pose_.pose.position.x, right_current_pose_.pose.position.y, right_current_pose_.pose.position.z, right_current_pose_.pose.orientation.x, right_current_pose_.pose.orientation.y, right_current_pose_.pose.orientation.z, right_current_pose_.pose.orientation.w);

    std::vector<std::string> armsJointNames = arms_.getActiveJoints();
    std::vector<double> armsJointValues = arms_.getCurrentJointValues();
    ROS_INFO("Reference Frame: %s", arms_.getPlanningFrame().c_str());
    for (std::size_t i = 0; i < armsJointNames.size(); i++)
    {
        ROS_INFO("Joint %s: %f", armsJointNames[i].c_str(), armsJointValues[i]);
    }
#endif

    // Makes sure all TFs exists before enabling all transformations in the callbacks
  	// Transform from base_link to world
	rotation_world_left_base_.setZero();
	// Transform from robotiq_ft_frame_id to tip_name
	rotation_tip_left_sensor_.setZero();
	rotation_world_right_base_.setZero();
	rotation_tip_right_sensor_.setZero();
    tf::TransformListener listener_arm_;
	while (!get_rotation_matrix(rotation_world_left_base_, listener_arm_, "world", "left_base"))
	{
		sleep(1);
	}
	
	while (!get_rotation_matrix(rotation_world_right_base_, listener_arm_, "world", "right_base"))
	{
		sleep(1);
	}
    while (!get_rotation_matrix(rotation_tip_left_sensor_, listener_arm_, "left_tool0", "left_robotiq_ft_frame_id"))
	{
		sleep(1);
	}
	
	while (!get_rotation_matrix(rotation_tip_right_sensor_, listener_arm_, "right_tool0", "right_robotiq_ft_frame_id"))
	{
		sleep(1);
	}
}
DualArmRobot::~DualArmRobot()
{
    // left_cartTraj_pub_thread.reset();
    // left_cartTraj_pub_thread->join();
    // right_cartTraj_pub_thread.reset();
    // right_cartTraj_pub_thread->join();
}
robot_state::RobotState DualArmRobot::getCurrentRobotState()
{
    // Request planning scene state using a service call (service name)
    planning_scene_monitorPtr_->requestPlanningSceneState("get_planning_scene");
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitorPtr_);
    ps->getCurrentStateNonConst().update();
    // Definition of a kinematic state (both joint and link)
    robot_state::RobotState current_state = ps->getCurrentState();
    return current_state;
}

/* Representation of a robot's state. This includes position, velocity, acceleration and effort. */
moveit_msgs::RobotState DualArmRobot::getCurrentRobotStateMsg()
{
    // http://docs.ros.org/melodic/api/moveit_msgs/html/msg/RobotState.html
    moveit_msgs::RobotState current_state_msg;
    // Convert a MoveIt! robot state to a robot state message.
    moveit::core::robotStateToRobotStateMsg(getCurrentRobotState(), current_state_msg);
    current_state_msg.is_diff = true;
    return current_state_msg;
}

// returns the Offset-Vector between both end effectors
// KDL::Frame represents a frame transformation in 3D space (rotation + translation)
// if V2 = Frame*V1 (V2 expressed in frame A, V1 expressed in frame B) then V2 = Frame.M*V1+Frame.p
// Frame.M contains columns that represent the axes of frame B wrt frame A
// Frame.p contains the origin of frame B expressed in frame A.
KDL::Frame DualArmRobot::getCurrentOffset()
{
    /* Expressed in World Frame */
    geometry_msgs::PoseStamped left_pose = left_.getCurrentPose(left_.getEndEffectorLink());
    geometry_msgs::PoseStamped right_pose = right_.getCurrentPose(right_.getEndEffectorLink());
    KDL::Frame left_frame;
    KDL::Frame right_frame;
    dual_arm_toolbox::Transform::transformPoseToKDL(left_pose.pose, left_frame);
    dual_arm_toolbox::Transform::transformPoseToKDL(right_pose.pose, right_frame);
    KDL::Frame offset_frame;
    /* The relative pose between two end-effectors, expressed in left_ee_link*/
    offset_frame = left_frame.Inverse() * right_frame;
    return offset_frame;
}
// calculates a trajectory for both arms based on the trajectory of one arm
// moveit_msgs::RobotTrajectory is a class object that contains two subclass, JointTrajectory and multi_dof_joint_trajectory
bool DualArmRobot::adaptTrajectory(moveit_msgs::RobotTrajectory left_trajectory,
                                   KDL::Frame offset_frame,
                                   moveit_msgs::RobotTrajectory &both_arms_trajectory,
                                   double jump_threshold)
{
    const std::vector<std::string> &right_joint_names = right_joint_model_group->getActiveJointModelNames();
    both_arms_trajectory = left_trajectory;
    for (unsigned int j = 0; j < right_joint_names.size(); j++)
    {
        both_arms_trajectory.joint_trajectory.joint_names.push_back(right_joint_names[j]);
    }

    // computing values for trajectory
    right_.setStartStateToCurrentState();
    for (unsigned int i = 0; i < left_trajectory.joint_trajectory.points.size(); i++)
    {
        std::vector<double> left_joint_values;
        left_joint_values.clear();
        for (unsigned int a = 0; a < left_trajectory.joint_trajectory.points[i].positions.size(); a++)
        {
            left_joint_values.push_back(left_trajectory.joint_trajectory.points[i].positions[a]);
        }
        // fk -> pos left
        KDL::Frame frame_pose_left;
        kinematic_statePtr->setJointGroupPositions(left_joint_model_group, left_joint_values);
        const Eigen::Affine3d &end_effector_pose_left = kinematic_statePtr->getGlobalLinkTransform(left_.getEndEffectorLink());
        tf::transformEigenToKDL(end_effector_pose_left, frame_pose_left);
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        geometry_msgs::Pose left_temp_pose;
        dual_arm_toolbox::Transform::transformKDLtoPose(frame_pose_left, left_temp_pose);
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        geometry_msgs::Pose right_temp_pose;
        // compute pos right
        KDL::Frame frame_pose_right = frame_pose_left * offset_frame;
        // dual_arm_toolbox::Transform::transformKDLtoPose(frame_pose_right, right_temp_pose);
        Eigen::Affine3d end_effector_pose_right;
        std::vector<double> right_joint_values;

        tf::transformKDLToEigen(frame_pose_right, end_effector_pose_right);

        bool found_ik = kinematic_statePtr->setFromIK(right_joint_model_group, end_effector_pose_right, 10, 0.1);
        if(found_ik){
            kinematic_statePtr->copyJointGroupPositions(right_joint_model_group, right_joint_values);
            for(std::size_t j=0; j<right_joint_values.size(); ++j){
                both_arms_trajectory.joint_trajectory.points[i].positions.push_back(right_joint_values[j]);
            }
            kinematic_statePtr->setJointGroupPositions(right_joint_model_group, right_joint_values);
        }else{
            ROS_WARN("Did not find ik solution for point %d !", i);
        }
    }
    // execution speed
    dual_arm_toolbox::TrajectoryProcessor::computeTimeFromStart(both_arms_trajectory, 0.4);
    // compute velocities for trajectory
    return dual_arm_toolbox::TrajectoryProcessor::computeVelocities(both_arms_trajectory, arms_);
}

bool DualArmRobot::switch_controller(std::string stop_name, std::string start_name, std::string ur_namespace)
{
    ROS_INFO("Switching controllers");
#ifndef OFFLINE
    // setup
    ros::ServiceClient srv_switch_controller = nh_.serviceClient<controller_manager_msgs::SwitchController>(ur_namespace + "/controller_manager/switch_controller");
    controller_manager_msgs::SwitchController switchController;
    switchController.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
    sleep(2);

    // stop
    switchController.request.stop_controllers.push_back(stop_name);
    bool success_stop = srv_switch_controller.call(switchController);
    ROS_INFO("Stopping controller %s %s", stop_name.c_str(), success_stop ? "SUCCEDED" : "FAILED");
    if (!success_stop)
        return false;

    // clear
    switchController.request.stop_controllers.clear();
    // start admittance controller
    switchController.request.BEST_EFFORT;
    switchController.request.start_controllers.push_back(start_name);
    bool success_start = srv_switch_controller.call(switchController);
    ROS_INFO("Starting controller %s %s", start_name.c_str(), success_start ? "SUCCEDED" : "FAILED");
    switchController.request.start_controllers.clear();

    // Switch controller in moveit-interface
    if (success_start)
    {
        if (left_controller_.compare(0, ur_namespace.size(), ur_namespace) == 0)
        {
            left_controller_ = ur_namespace + "/" + start_name;
        }
        if (right_controller_.compare(0, ur_namespace.size(), ur_namespace) == 0)
        {
            right_controller_ = ur_namespace + "/" + start_name;
        }
    }

    return success_start;
#endif
#ifdef OFFLINE
    return true;
#endif
}

bool DualArmRobot::graspMove(double distance, bool avoid_collisions, bool use_left, bool use_right, float scale)
{
    bool try_step = false;
    double fraction;

    // move closer with right and left by using cartesian path
    if (distance > 0)
        ROS_INFO("Moving towards object");
    if (distance < 0)
        ROS_INFO("Moving away from object");
    moveit_msgs::RobotTrajectory left_trajectory;
    moveit_msgs::RobotTrajectory right_trajectory;

    // right
    if (use_right) try_step = true;
    while(try_step && ros::ok()) {
        right_.setStartStateToCurrentState();
        std::vector<geometry_msgs::Pose> right_waypoints;
        geometry_msgs::Pose right_waypoint = right_.getCurrentPose(right_.getEndEffectorLink()).pose;
        right_waypoints.push_back(right_waypoint);

        // transform distance vector
        KDL::Vector right_vec_d;  // distance vector
        KDL::Frame right_p_eef;   // endeffector frame
        dual_arm_toolbox::Transform::transformPoseToKDL(right_waypoint, right_p_eef);
        right_vec_d.x(0);
        right_vec_d.y(0);
        right_vec_d.z(distance);
        right_vec_d = right_p_eef.M * right_vec_d;     // Rotate distance vector

        // calculate waypoint
        right_waypoint.position.x = right_waypoint.position.x + right_vec_d.x();
        right_waypoint.position.y = right_waypoint.position.y + right_vec_d.y();
        right_waypoint.position.z = right_waypoint.position.z + right_vec_d.z();

        right_waypoints.push_back(right_waypoint);
        fraction = right_.computeCartesianPath(right_waypoints, 0.001, 0.0, right_trajectory, avoid_collisions);
        if (fraction < 0.9) {
            ROS_WARN("Right arm cartesian path. (%.2f%% achieved)", fraction * 100.0);
            try_step = try_again_question();
            if (!try_step) return false;
        }
        try_step = false; 
        dual_arm_toolbox::TrajectoryProcessor::clean(right_trajectory);
        dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(right_trajectory, scale);
    }
    if (use_left) try_step = true;
    while (try_step && ros::ok())
    {
        /* Update the robot state */
        current_dual_arm_robotstate_msg_ = getCurrentRobotStateMsg();
        left_.setStartState(current_dual_arm_robotstate_msg_);

        std::vector<geometry_msgs::Pose> left_waypoints;
        /* Expressed in world frame */
        left_current_pose_ = left_.getCurrentPose(left_.getEndEffectorLink());
        geometry_msgs::Pose left_waypoint = left_current_pose_.pose;
        left_waypoints.push_back(left_waypoint);

        // transform distance vector
        KDL::Vector left_vec_d; // distance vector
        KDL::Frame left_p_eef;  // endeffector frame
        dual_arm_toolbox::Transform::transformPoseToKDL(left_waypoint, left_p_eef);
        left_vec_d.x(0);
        left_vec_d.y(0);
        left_vec_d.z(distance);
        left_vec_d = left_p_eef.M * left_vec_d; // Rotate distance vector
        

        // calculate waypoint
        left_waypoint.position.x = left_waypoint.position.x + left_vec_d.x();
        left_waypoint.position.y = left_waypoint.position.y + left_vec_d.y();
        left_waypoint.position.z = left_waypoint.position.z + left_vec_d.z();

        left_waypoints.push_back(left_waypoint);
        
        /* Compute a Cartesian path that follows specified waypoints with a step size of at most eef_step (0.001) meters between end effector configurations  */
        fraction = left_.computeCartesianPath(left_waypoints, 0.001, 0.0, left_trajectory, avoid_collisions);
        if (fraction < 0.9)
        {
            ROS_WARN("Left arm cartesian path. (%.2f%% achieved)", fraction * 100.0);
            try_step = try_again_question();
            if (!try_step)
                return false;
        } 
        try_step = false; 
        dual_arm_toolbox::TrajectoryProcessor::clean(left_trajectory);
        dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(left_trajectory, scale);
    }
    if(use_left && use_right)
    {
        moveit_msgs::RobotTrajectory both_arms_trajectory;
        dual_arm_toolbox::TrajectoryProcessor::fuse(both_arms_trajectory, left_trajectory, right_trajectory);
        moveit::planning_interface::MoveGroupInterface::Plan arms_plan;
        dual_arm_toolbox::TrajectoryProcessor::clean(both_arms_trajectory);
        arms_plan.trajectory_ = both_arms_trajectory;
        bool success = execute(arms_plan);
        if (!success){
            ROS_ERROR("Both arm's trajectory failed! ");
            return false;
        }
            
    } else if(use_left){
        
        moveit::planning_interface::MoveGroupInterface::Plan left_plan;
        left_plan.trajectory_ = left_trajectory;
        bool success = execute(left_plan);
        if (!success){
            ROS_ERROR("Left arm's trajectory failed! ");
            return false;
        }
       
    } else {
            moveit::planning_interface::MoveGroupInterface::Plan right_plan;
            right_plan.trajectory_ = right_trajectory;
            bool success = execute(right_plan);
        if (!success){
            ROS_ERROR("Right arm's trajectory failed! ");
            return false;
        }
    }
    return true;
}

bool DualArmRobot::try_again_question()
{
    // automatic retry
    if (ros::ok())
    {
        ROS_WARN("Trying this step again. Press Ctr+C to abort.");
        return true;
    }
    return false;
}

bool DualArmRobot::pickBox(std::string object_id, geometry_msgs::Vector3Stamped &lift_direction)
{
    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;
    // compute cartesian Path for left
    left_.setStartStateToCurrentState();
    moveit_msgs::RobotTrajectory left_trajectory;
    try_step = true;
    while (try_step && ros::ok())
    {
        left_.setPoseReferenceFrame(lift_direction.header.frame_id);
        std::vector<geometry_msgs::Pose> left_waypoints;
        geometry_msgs::Pose left_waypoint = left_.getCurrentPose(left_.getEndEffectorLink()).pose;
        left_waypoints.push_back(left_waypoint);
        left_waypoint.position.x = left_waypoint.position.x + lift_direction.vector.x;
        left_waypoint.position.y = left_waypoint.position.y + lift_direction.vector.y;
        left_waypoint.position.z = left_waypoint.position.z + lift_direction.vector.z;
        left_waypoints.push_back(left_waypoint);
        double fraction = left_.computeCartesianPath(left_waypoints, 0.01, 0.0, left_trajectory, false);
        if (fraction < 1)
        {
            ROS_WARN("Left arm cartesian path. (%.2f%% achieved)", fraction * 100.0);
            ROS_WARN("Fraction is less than 100%%. Insufficient for dual-arm configuration. Pick can not be executed");
            try_step = try_again_question();
            if (!try_step)
            {
                return false;
            }
        }
        else
            try_step = false;
    }
    // get both arms trajectory
    arms_offset_ = getCurrentOffset();
    moveit_msgs::RobotTrajectory both_arms_trajectory;
    // 0.001m 400ms =>0.0025m/s
    if (adaptTrajectory(left_trajectory, arms_offset_, both_arms_trajectory)){
        dual_arm_toolbox::TrajectoryProcessor::clean(both_arms_trajectory);
        dual_arm_toolbox:: TrajectoryProcessor::scaleTrajectorySpeed(both_arms_trajectory, 0.15);
    }
    else
    {
        ROS_WARN("Problem adapting trajectory");
        allowedArmCollision(false, object_id);
        return false;
    }
    // get plan from trajectory
    moveit::planning_interface::MoveGroupInterface::Plan both_arms_plan;
    both_arms_plan.trajectory_ = both_arms_trajectory;
    both_arms_plan.start_state_ = getCurrentRobotStateMsg();
    // Grasp by switching controller and wait for contact while visualizing plan
    if (!switch_controller("vel_based_pos_traj_controller", "vel_based_admittance_traj_controller", "right"))
        ROS_WARN("failed switching controller");
    // if (!switch_controller("vel_based_pos_traj_controller", "vel_based_admittance_traj_controller", "left"))
    //     ROS_WARN("failed switching controller");
    execute(both_arms_plan);
    return true;
}
bool DualArmRobot::placeBox(std::string object_id, geometry_msgs::Vector3Stamped &place_direction)
{
    pickBox(object_id, place_direction);
    // un-grasp
    if (!switch_controller("vel_based_admittance_traj_controller", "vel_based_pos_traj_controller", "right"))
        ROS_WARN("failed switching controller");
    // if (!switch_controller("vel_based_admittance_traj_controller", "vel_based_pos_traj_controller", "left"))
    //     ROS_WARN("failed switching controller");
    // graspMove(-0.02, false, true, true);
    return true;
}
bool DualArmRobot::linearMoveParallel(geometry_msgs::Vector3Stamped direction,
                                      std::string object_id,
                                      double traj_scale,
                                      bool avoid_collisions)
{
    // compute cartesian Path for left arm
    moveit_msgs::RobotTrajectory left_trajectory;
    bool try_step = true;
    while (try_step && ros::ok())
    {
        left_.setPoseReferenceFrame(direction.header.frame_id);
        left_.setStartState(current_dual_arm_robotstate_msg_);
        allowedArmCollision(true, object_id);
        std::vector<geometry_msgs::Pose> left_waypoints;
        geometry_msgs::Pose left_waypoint = left_current_pose_.pose;
        left_waypoints.push_back(left_waypoint);
        left_waypoint.position.x = left_waypoint.position.x + direction.vector.x;
        left_waypoint.position.y = left_waypoint.position.y + direction.vector.y;
        left_waypoint.position.z = left_waypoint.position.z + direction.vector.z;
        left_waypoints.push_back(left_waypoint);
        double fraction = left_.computeCartesianPath(left_waypoints, 0.002, 0, left_trajectory, false);
        if (fraction < 1)
        {
            ROS_INFO("Left arm cartesian path. (%.2f%% achieved)", fraction * 100.0);
            ROS_INFO("Fraction is less than 100%%. Insufficient for dual-arm configuration. Linear parallel move can not be executed");
            try_step = try_again_question();
            if (!try_step)
            {
                return false;
            }
        }
        else
            try_step = false;
    }

    dual_arm_toolbox::TrajectoryProcessor::clean(left_trajectory);

    // get both arms trajectory
    arms_offset_ = getCurrentOffset();
    moveit_msgs::RobotTrajectory both_arms_trajectory;
    if (!adaptTrajectory(left_trajectory, arms_offset_, both_arms_trajectory, 0.5))
    {
        ROS_WARN("Problem adapting trajectory");
        return false;
    }
    dual_arm_toolbox::TrajectoryProcessor::clean(both_arms_trajectory);

    // scale trajectory
    dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(both_arms_trajectory, traj_scale);

    // get plan from trajectory
    moveit::planning_interface::MoveGroupInterface::Plan both_arms_plan;
    both_arms_plan.trajectory_ = both_arms_trajectory;
    // both_arms_plan.start_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;
    execute(both_arms_plan);
    return true;
}


bool DualArmRobot::pushPlaceBox(std::string object_id, geometry_msgs::PoseStamped box_pose, geometry_msgs::Vector3 direction)
{
    ROS_INFO("Starting Push Place Box sequence");
    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;
    dual_arm_demonstrator_iml::SceneManager sceneManager(nh_);

    // calculate left pose from box goal pose
    geometry_msgs::PoseStamped left_target_pose;
    left_target_pose = box_pose;
    KDL::Rotation left_rot;
    left_rot = left_rot.Quaternion(left_target_pose.pose.orientation.x, left_target_pose.pose.orientation.y, left_target_pose.pose.orientation.z, left_target_pose.pose.orientation.w);
    left_rot.DoRotY(-3.14 / 2);
    KDL::Vector left_shift;
    left_shift.z(-sceneManager.box_.dimensions[0] / 2);
    left_shift = left_rot * left_shift;
    left_target_pose.pose.position.x = left_target_pose.pose.position.x - direction.x + left_shift.x();
    left_target_pose.pose.position.y = left_target_pose.pose.position.y - direction.y + left_shift.y();
    left_target_pose.pose.position.z = left_target_pose.pose.position.z - direction.z + left_shift.z();
    left_rot.GetQuaternion(left_target_pose.pose.orientation.x, left_target_pose.pose.orientation.y, left_target_pose.pose.orientation.z, left_target_pose.pose.orientation.w);

    // left target position for placing
    // if (!moveObject(object_id, left_target_pose, 0.1))
    //     return false;

    // release clamp
    // switch controller
    if (!switch_controller("left_vel_based_admittance_traj_controller", "vel_based_pos_traj_controller", "left"))
        ROS_WARN("failed switching controller");
    sleep(0.5); // to be sure robot is at goal position

    // correct pose. Important for box to be at the right place in simulation
    geometry_msgs::Vector3Stamped correct_vec;
    correct_vec.header.frame_id = left_current_pose_.header.frame_id;
    geometry_msgs::PoseStamped curr_pose = left_.getCurrentPose(left_.getEndEffectorLink());
    correct_vec.vector.x = left_current_pose_.pose.position.x - curr_pose.pose.position.x;
    correct_vec.vector.y = left_current_pose_.pose.position.y - curr_pose.pose.position.y;
    correct_vec.vector.z = left_current_pose_.pose.position.z - curr_pose.pose.position.z;
    left_current_pose_ = curr_pose;
    current_dual_arm_robotstate_msg_ = getCurrentRobotStateMsg();
    linearMove(correct_vec, false, true, false);

    // detach object
    left_.detachObject(object_id);
    current_dual_arm_robotstate_msg_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // un-grasp left
    graspMove(-0.05, false, true, false);

    // move left to prepare for push
    try_step = true;
    while (try_step && ros::ok())
    {
        ROS_INFO("ENTE1");
        left_.setPoseReferenceFrame("left_ee_0");
        left_.setStartStateToCurrentState();
        // waypoints
        std::vector<geometry_msgs::Pose> left_waypoints;
        geometry_msgs::Pose left_waypoint;
        left_waypoint.position.x = 0;
        left_waypoint.position.y = 0;
        left_waypoint.position.z = 0;
        left_waypoint.orientation.x = 0;
        left_waypoint.orientation.y = 0;
        left_waypoint.orientation.z = 0;
        left_waypoint.orientation.w = 1;
        left_waypoints.push_back(left_waypoint);
        left_waypoint.position.x = left_waypoint.position.x;
        left_waypoint.position.y = left_waypoint.position.y + sceneManager.box_.dimensions[1] / 2 + 0.05;
        left_waypoint.position.z = left_waypoint.position.z;
        left_waypoints.push_back(left_waypoint);

        moveit_msgs::RobotTrajectory left_trajectory;
        double fraction = left_.computeCartesianPath(left_waypoints, 0.001, 0.0, left_trajectory, true);
        if (fraction < 0.9)
        {
            ROS_WARN("Left arm cartesian path. (%.2f%% achieved)", fraction * 100.0);
            try_step = try_again_question();
            if (!try_step)
                return false;
        }
        else
        {
            dual_arm_toolbox::TrajectoryProcessor::clean(left_trajectory);
            moveit::planning_interface::MoveGroupInterface::Plan left_plan;
            left_plan.trajectory_ = left_trajectory;
            execute(left_plan);
            left_.setPoseReferenceFrame("world");
            try_step = false;
        }
    }
    try_step = true;
    while (try_step && ros::ok())
    {
        ROS_INFO("ENTE2");
        left_.setPoseReferenceFrame("left_ee_0");
        left_.setStartStateToCurrentState();
        // waypoints
        std::vector<geometry_msgs::Pose> left_waypoints;
        geometry_msgs::Pose left_waypoint;
        left_waypoint.position.x = 0;
        left_waypoint.position.y = 0;
        left_waypoint.position.z = 0;
        left_waypoint.orientation.x = 0;
        left_waypoint.orientation.y = 0;
        left_waypoint.orientation.z = 0;
        left_waypoint.orientation.w = 1;
        left_waypoints.push_back(left_waypoint);

        left_rot = left_rot.Quaternion(left_waypoint.orientation.x, left_waypoint.orientation.y, left_waypoint.orientation.z, left_waypoint.orientation.w);
        left_rot.DoRotX(3.14 / 2);
        left_rot.GetQuaternion(left_waypoint.orientation.x, left_waypoint.orientation.y, left_waypoint.orientation.z, left_waypoint.orientation.w);
        left_waypoint.position.z = left_waypoint.position.z + 0.18;
        left_waypoints.push_back(left_waypoint);

        moveit_msgs::RobotTrajectory left_trajectory;
        double fraction = left_.computeCartesianPath(left_waypoints, 0.001, 0.0, left_trajectory, true);
        if (fraction < 0.9)
        {
            ROS_WARN("Left arm cartesian path. (%.2f%% achieved)", fraction * 100.0);
            try_step = try_again_question();
            if (!try_step)
                return false;
        }
        else
        {
            dual_arm_toolbox::TrajectoryProcessor::clean(left_trajectory);
            moveit::planning_interface::MoveGroupInterface::Plan left_plan;
            left_plan.trajectory_ = left_trajectory;
            execute(left_plan);
            left_.setPoseReferenceFrame("world");
            try_step = false;
        }
    }

    // move closer
    graspMove(0.05, false, true, false);

    // attach object again to left and update State Msg
    left_.attachObject(object_id, left_.getEndEffectorLink());
    current_dual_arm_robotstate_msg_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // push box into goal position  //TODO: method linear move with vector.
    geometry_msgs::Vector3Stamped directionStamped;
    directionStamped.header.frame_id = "world";
    // transform direction vector into frame "world"
    KDL::Rotation shelf_table_rot;
    shelf_table_rot.DoRotZ(-3.14 / 4);
    KDL::Vector direction_vec;
    direction_vec.x(direction.x);
    direction_vec.y(direction.y);
    direction_vec.z(direction.z);
    direction_vec = shelf_table_rot * direction_vec;
    directionStamped.vector.x = direction_vec.x();
    directionStamped.vector.y = direction_vec.y();
    directionStamped.vector.z = direction_vec.z();
    linearMove(directionStamped, false, true, false);

    // detach object
    left_.detachObject(object_id);
    current_dual_arm_robotstate_msg_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // moving away from object using the same vector in opposite direction
    directionStamped.vector.x = -directionStamped.vector.x;
    directionStamped.vector.y = -directionStamped.vector.y;
    directionStamped.vector.z = -directionStamped.vector.z;
    linearMove(directionStamped, false, true, true);

    return true;
}
// Move the object from the current position to the target position defined by left_pose in the air (before placing it down)
bool DualArmRobot::moveObject(std::string object_id, const std::vector< geometry_msgs::Pose > &left_waypoints, double scale)
{
    bool try_step = true;
    double fraction = 0.0;
    moveit::planning_interface::MoveItErrorCode error;
    moveit_msgs::RobotTrajectory left_trajectory;
    moveit_msgs::RobotTrajectory both_arms_trajectory;
    
    while (try_step && ros::ok())
    {
        fraction = left_.computeCartesianPath(left_waypoints, 0.0015, 0, left_trajectory, true, &error);
        if (fraction < 0.9)
        {
            ROS_WARN("Left arm cartesian path. (%.2f%% achieved), error code %d", fraction * 100.0, error.val);
            try_step = try_again_question();
            if (!try_step)
                return false;
        }

        if (!adaptTrajectory(left_trajectory, arms_offset_, both_arms_trajectory, 0.5))
        {
            ROS_WARN("Problem adapting trajectory");
            try_step = try_again_question();
            if (!try_step)
                return false;
        }else{
            try_step = false;
        }
    }

    dual_arm_toolbox::TrajectoryProcessor::clean(both_arms_trajectory);
    dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(both_arms_trajectory, scale);

    // get plan from trajectory
    moveit::planning_interface::MoveGroupInterface::Plan both_arms_plan;
    both_arms_plan.trajectory_ = both_arms_trajectory;
    // both_arms_plan.start_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;
    execute(both_arms_plan);
    // allowedArmCollision(false, object_id);
    return true;
}

bool DualArmRobot::planMoveObject(std::string object_id, const std::vector< geometry_msgs::Pose > &left_waypoints, double scale)
{
    ROS_INFO("Moving object with both arms (planning only)");
    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;
    moveit_msgs::RobotTrajectory left_trajectory;
    moveit_msgs::RobotTrajectory both_arms_trajectory;
    bool adapt_error;

    // evaluation
    ros::Time before_planning_left;
    ros::Time after_planning_left;
    ros::Duration planning_time_left;

    ros::Time before_planning_adaption;
    ros::Time after_planning_adaption;
    ros::Duration planning_time_adaption;

    ros::Time before_planning_loop;
    ros::Time after_planning_loop;
    ros::Duration planning_time_loop;

    unsigned int loops = 0;
    unsigned long int traj_steps;

    // plan left plan
    allowedArmCollision(true, object_id);
    try_step = true;
    moveit::planning_interface::MoveGroupInterface::Plan left_plan;
    before_planning_loop = ros::Time::now();
    allowedArmCollision(true, object_id);
    while (try_step && ros::ok())
    {
        loops++;
        adapt_error = false;
       

        before_planning_left = ros::Time::now();
        double fraction = left_.computeCartesianPath(left_waypoints, 0.002, 0, left_trajectory, true, &error);
        if (fraction < 0.9)
        {
            ROS_WARN("Left arm cartesian path. (%.2f%% achieved), error code %d", fraction * 100.0, error.val);
            try_step = try_again_question();
            if (!try_step)
                return false;
        }
        after_planning_left = ros::Time::now();

        // get both arms trajectory
        before_planning_adaption = ros::Time::now();
        if (!adaptTrajectory(left_trajectory, arms_offset_, both_arms_trajectory, 0.5))
        {
            ROS_WARN("Problem adapting trajectory");
            try_step = try_again_question();
            if (!try_step)
                return false;
        }else{
            try_step = false;
        }
        after_planning_adaption = ros::Time::now();
       
    }
    after_planning_loop = ros::Time::now();
    dual_arm_toolbox::TrajectoryProcessor::clean(both_arms_trajectory);
    dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(both_arms_trajectory, scale);

    // get plan from trajectory
    moveit::planning_interface::MoveGroupInterface::Plan both_arms_plan;
    both_arms_plan.trajectory_ = both_arms_trajectory;
    // both_arms_plan.start_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;
    // execute(both_arms_plan);

    // evaluation
    planning_time_left = after_planning_left - before_planning_left;
    traj_steps = both_arms_trajectory.joint_trajectory.points.size();
    std::cout << both_arms_trajectory.joint_trajectory.points.size();
    planning_time_adaption = after_planning_adaption - before_planning_adaption;
    planning_time_loop = after_planning_loop - before_planning_loop;

    ROS_INFO(":::::: VALUES EVALUATION ::::::");
    ROS_INFO("Planning Time for left took: %f nsec", planning_time_left.toSec());
    ROS_INFO("Planning Time for adaption took: %f nsec", planning_time_adaption.toSec());
    ROS_INFO("Trajectory has %ld waypoints", traj_steps);
    ROS_INFO("Planning Time for loop took: %f nsec", planning_time_loop.toSec());
    ROS_INFO("Planning loop count: %d steps", loops);

    // visualize plan
    dual_arm_toolbox::TrajectoryProcessor::visualizePlan(both_arms_plan, 0);

    return true;
}

void DualArmRobot::allowedArmCollision(bool enable, std::string left_attachedObject)
{
    collision_detection::AllowedCollisionMatrix acm = planningScenePtr->getAllowedCollisionMatrix();
    moveit_msgs::PlanningScene planningSceneMsg;

    ros::Publisher planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    // Robot links and attached object
    // Be careful of the sequence, make reference to ur_kinematics
    std::vector<std::string> left_links;
    left_links.push_back("left_ee_link");
    left_links.push_back("left_forearm_link");
    left_links.push_back("left_tool0");
    left_links.push_back("left_upper_arm_link");
    left_links.push_back("left_wrist_1_link");
    left_links.push_back("left_wrist_2_link");
    left_links.push_back("left_wrist_3_link");
    // left_links.push_back("left_ee_0");
    // left_links.push_back("left_ee_frame");
    // left_links.push_back("left_ee_spring");
    left_links.push_back(left_attachedObject.c_str());

    std::vector<std::string> right_links;
    right_links.push_back("right_ee_link");
    right_links.push_back("right_forearm_link");
    right_links.push_back("right_tool0");
    right_links.push_back("right_upper_arm_link");
    right_links.push_back("right_wrist_1_link");
    right_links.push_back("right_wrist_2_link");
    right_links.push_back("right_wrist_3_link");
    // right_links.push_back("right_ee_0");
    // right_links.push_back("right_ee_frame");
    // right_links.push_back("right_ee_spring");

    acm.setEntry(left_links, right_links, enable); //Set an entry corresponding to all possible pairs between two sets of elements.

    // publish scene diff
    acm.getMessage(planningSceneMsg.allowed_collision_matrix);
    planningSceneMsg.is_diff = true;
    planning_scene_diff_publisher.publish(planningSceneMsg);
    ROS_INFO("Allowed collision between robot arms %s", enable ? "ENABLED" : "DISABLED");
    // sleep(2);
}

bool DualArmRobot::linearMove(geometry_msgs::Vector3Stamped direction,
                              bool avoid_collisions,
                              bool use_left,
                              bool use_right)
{

    bool try_step;
    double fraction;
    moveit_msgs::RobotTrajectory right_trajectory;
    moveit_msgs::RobotTrajectory left_trajectory;
    int try_count = 0;
    // left arm
    if (use_left)
        try_step = true;
    while (try_step && ros::ok())
    {
        ROS_INFO("Let left arm move straight line");
        left_.setPoseReferenceFrame(direction.header.frame_id);
        /* Update the robot state */
        current_dual_arm_robotstate_msg_ = getCurrentRobotStateMsg();
        left_.setStartState(current_dual_arm_robotstate_msg_);
        // fk service client setup
        ros::ServiceClient fk_client = nh_.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");
        // A service definition for a standard forward kinematics service
        // The frame_id in the header messages is the frame in which the forward kinematics poses will be returned.
        moveit_msgs::GetPositionFK fk_msg;
        fk_msg.request.header.frame_id = direction.header.frame_id;
        fk_msg.request.fk_link_names.push_back(left_.getEndEffectorLink());
        fk_msg.request.robot_state = current_dual_arm_robotstate_msg_;
        fk_client.call(fk_msg.request, fk_msg.response);
        // get virtual pose from virtual state
        if (fk_msg.response.error_code.val != 1)
        {
            ROS_WARN("fk request error");
            return false;
        }
        geometry_msgs::PoseStamped left_start_pose = fk_msg.response.pose_stamped[0];

        // waypoints
        std::vector<geometry_msgs::Pose> left_waypoints;
        geometry_msgs::Pose left_waypoint = left_start_pose.pose;
        left_waypoints.push_back(left_waypoint);
#ifdef NO_CHANCE_THAT_THIS_SYMBOL_WILL_EVER_EXIST
        ROS_INFO("Linear Linear move start point( %f %f %f)\n",
                 left_waypoint.position.x,
                 left_waypoint.position.y,
                 left_waypoint.position.z);
#endif
        left_waypoint.position.x = left_waypoint.position.x + direction.vector.x;
        left_waypoint.position.y = left_waypoint.position.y + direction.vector.y;
        left_waypoint.position.z = left_waypoint.position.z + direction.vector.z;
#ifdef NO_CHANCE_THAT_THIS_SYMBOL_WILL_EVER_EXIST
        ROS_INFO("Left Linear move target point( %f %f %f)\n",
                 left_waypoint.position.x,
                 left_waypoint.position.y,
                 left_waypoint.position.z);
#endif
        left_waypoints.push_back(left_waypoint);
        fraction = left_.computeCartesianPath(left_waypoints, 0.001, 0.0, left_trajectory, avoid_collisions);
        if (fraction < 0.9)
        {
            ROS_WARN("Left arm cartesian path. (%.2f%% achieved)", fraction * 100.0);
            try_step = try_again_question();
            if (!try_step)
                return false;
        }
        else
        {
            // dual_arm_toolbox::TrajectoryProcessor::clean(left_trajectory);
            // moveit::planning_interface::MoveGroupInterface::Plan left_plan;
            // left_plan.trajectory_ = left_trajectory;
            // bool success = execute(left_plan);
            // if(!success)
            //     ROS_WARN("Left arm trajectory failed! ");
            try_step = false;
        }
        // sleep(1);
    }
    // right arm
    if (use_right)
        try_step = true;
    while (try_step && ros::ok())
    {
        ROS_INFO("Let right arm move straight line");
        right_.setPoseReferenceFrame(direction.header.frame_id);
        current_dual_arm_robotstate_msg_ = getCurrentRobotStateMsg();
        right_.setStartState(current_dual_arm_robotstate_msg_);

        ros::ServiceClient fk_client = nh_.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");
        // A service definition for a standard forward kinematics service
        // The frame_id in the header messages is the frame in which the forward kinematics poses will be returned.
        moveit_msgs::GetPositionFK fk_msg;
        fk_msg.request.header.frame_id = direction.header.frame_id;
        fk_msg.request.fk_link_names.push_back(right_.getEndEffectorLink());
        fk_msg.request.robot_state = current_dual_arm_robotstate_msg_;
        fk_client.call(fk_msg.request, fk_msg.response);
        if (fk_msg.response.error_code.val != 1)
        {
            ROS_WARN("fk request error");
            return false;
        }
        geometry_msgs::PoseStamped right_start_pose = fk_msg.response.pose_stamped[0];

        // waypoints
        std::vector<geometry_msgs::Pose> right_waypoints;
        geometry_msgs::Pose right_waypoint = right_start_pose.pose;

        right_waypoints.push_back(right_waypoint);
        ROS_INFO("Right Linear move start point( %f %f %f)",
                 right_waypoint.position.x,
                 right_waypoint.position.y,
                 right_waypoint.position.z);
        // calculate waypoint
        right_waypoint.position.x = right_waypoint.position.x + direction.vector.x;
        right_waypoint.position.y = right_waypoint.position.y + direction.vector.y;
        right_waypoint.position.z = right_waypoint.position.z + direction.vector.z;
        ROS_INFO("Right Linear move target point( %f %f %f)\n",
                 right_waypoint.position.x,
                 right_waypoint.position.y,
                 right_waypoint.position.z);

        right_waypoints.push_back(right_waypoint);

        fraction = right_.computeCartesianPath(right_waypoints, 0.001, 0.0, right_trajectory, avoid_collisions);
        if (fraction < 0.9)
        {
            ROS_WARN("Right arm cartesian path. (%.2f%% achieved)", fraction * 100.0);
            try_step = try_again_question();
            if (!try_step)
                return false;
        }
        else
        {
            // moveit::planning_interface::MoveGroupInterface::Plan right_plan;
            // dual_arm_toolbox::TrajectoryProcessor::clean(right_trajectory);
            // right_plan.trajectory_ = right_trajectory;
            // bool success = execute(right_plan);
            // if(!success)
            //     ROS_WARN("Right arm trajectory failed! ");
            try_step = false;
        }
    }
    moveit_msgs::RobotTrajectory both_arms_trajectory;
    dual_arm_toolbox::TrajectoryProcessor::clean(left_trajectory);
    dual_arm_toolbox::TrajectoryProcessor::clean(right_trajectory);
    dual_arm_toolbox::TrajectoryProcessor::fuse(both_arms_trajectory, left_trajectory, right_trajectory);
    moveit::planning_interface::MoveGroupInterface::Plan arms_plan;
    dual_arm_toolbox::TrajectoryProcessor::clean(both_arms_trajectory);
    dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(both_arms_trajectory, 0.4);
    arms_plan.trajectory_ = both_arms_trajectory;
    bool success = execute(arms_plan);
    if (!success)
        ROS_WARN("Both arm's trajectory failed! ");
}

bool DualArmRobot::execute(moveit::planning_interface::MoveGroupInterface::Plan plan)
{
    ros::Time before_execute;
    ros::Duration execute_time;
    ros::Time after_execute;
    before_execute = ros::Time::now();
    moveit::planning_interface::MoveGroupInterface::Plan plan_left;
    moveit::planning_interface::MoveGroupInterface::Plan plan_right;
    dual_arm_toolbox::TrajectoryProcessor::visualizePlan(plan, 0);
    dual_arm_toolbox::TrajectoryProcessor::split(plan.trajectory_, plan_left.trajectory_, plan_right.trajectory_, "left", "right");
    dual_arm_toolbox::TrajectoryProcessor::clean(plan_left.trajectory_);
    dual_arm_toolbox::TrajectoryProcessor::clean(plan_right.trajectory_);
    plan.start_state_.is_diff = true;
    plan_left.start_state_.is_diff = true;
    plan_right.start_state_.is_diff = true;
    // These are two action client, whose names are, respectively, left_controller_ and right_controller_
    // action namespace is follow_joint_trajectory
    moveit_simple_controller_manager::FollowJointTrajectoryControllerHandle handle_left(left_controller_, "follow_joint_trajectory");
    moveit_simple_controller_manager::FollowJointTrajectoryControllerHandle handle_right(right_controller_, "follow_joint_trajectory");

    // for both arms trajectories: because in planning each arm was not aware of the other there is a collision check before executing the trajectory
    // TODO: put in again later
    ros::Time time_send_cmd = ros::Time::now();
    
    
    if ((plan_left.trajectory_.joint_trajectory.joint_names.size() > 0) && (plan_right.trajectory_.joint_trajectory.joint_names.size() > 0))
    {
        // check trajectory for collisions
        plan.start_state_.is_diff = true;
        bool isValid = planningScenePtr->isPathValid(plan.start_state_, plan.trajectory_, "arms");
        if (!isValid)
        {
            ROS_ERROR("Path is invalid. Execution aborted");
            // dual_arm_toolbox::TrajectoryProcessor::PrintTrajectory(plan.trajectory_);
            // return false;
        }  
        ROS_INFO("Trajectory header stamp : %f, left  %ld points, right %ld points.", 
            time_send_cmd.toSec(), 
            plan_left.trajectory_.joint_trajectory.points.size(), 
            plan_right.trajectory_.joint_trajectory.points.size());

        std::shared_ptr<std::thread> left_cartTraj_pub_thread = std::make_shared<std::thread>(
                                        std::bind(&DualArmRobot::publishPlanCartTrajectory, this, 
                                                    left_.getEndEffectorLink(),
                                                    left_joint_model_group,
                                                    plan_left.trajectory_));

        
        std::shared_ptr<std::thread> right_cartTraj_pub_thread = std::make_shared<std::thread>(
                                        std::bind(&DualArmRobot::publishPlanCartTrajectory, this, 
                                        right_.getEndEffectorLink(),
                                        right_joint_model_group,
                                        plan_right.trajectory_));
        
        
        if(handle_left.sendTrajectory(plan_left.trajectory_) && handle_right.sendTrajectory(plan_right.trajectory_)) {
            if(!handle_left.waitForExecution() || !handle_right.waitForExecution()){
                ROS_ERROR("The execution is not complete!!");
                return false;
            }
        }else{
            ROS_ERROR("The controller cannot accept the trajectory.!!");
            return false;
        }
        // http://docs.ros.org/jade/api/moveit_simple_controller_manager/html/action__based__controller__handle_8h_source.html#l00106
        left_cartTraj_pub_thread->join();
        right_cartTraj_pub_thread->join();
       
    } else if (plan_left.trajectory_.joint_trajectory.joint_names.size() > 0)
    {
        std::shared_ptr<std::thread> left_cartTraj_pub_thread = std::make_shared<std::thread>(std::bind(&DualArmRobot::publishPlanCartTrajectory, this, 
                                                                        left_.getEndEffectorLink(),
                                                                        left_joint_model_group,
                                                                        plan_left.trajectory_));


        
        if(handle_left.sendTrajectory(plan_left.trajectory_)) {
            if(!handle_left.waitForExecution()){
                ROS_ERROR("The execution is not complete!!");
                return false;
            }
        }else{
            ROS_ERROR("The controller cannot accept the trajectory.!!");
            return false;
        }
    } else {
        std::shared_ptr<std::thread> right_cartTraj_pub_thread = std::make_shared<std::thread>(
                                        std::bind(&DualArmRobot::publishPlanCartTrajectory, this, 
                                                                        right_.getEndEffectorLink(),
                                                                        right_joint_model_group,
                                                                        plan_right.trajectory_));
        
        if(handle_right.sendTrajectory(plan_right.trajectory_)) {
            if(!handle_right.waitForExecution()){
                ROS_ERROR("The execution is not complete!!");
                return false;
            }
        }else{
            ROS_ERROR("The controller cannot accept the trajectory.!!");
            return false;
        }
    }
    dual_arm_toolbox::TrajectoryProcessor::publishPlanTrajectory(plan, 1);
    if (plan_left.trajectory_.joint_trajectory.joint_names.size())
    {
        current_dual_arm_robotstate_msg_.is_diff = true;
        current_dual_arm_robotstate_msg_.joint_state.effort = plan_left.trajectory_.joint_trajectory.points[plan_left.trajectory_.joint_trajectory.points.size() - 1].effort;
        current_dual_arm_robotstate_msg_.joint_state.header = plan_left.trajectory_.joint_trajectory.header;
        current_dual_arm_robotstate_msg_.joint_state.name = plan_left.trajectory_.joint_trajectory.joint_names;
        current_dual_arm_robotstate_msg_.joint_state.position = plan_left.trajectory_.joint_trajectory.points[plan_left.trajectory_.joint_trajectory.points.size() - 1].positions;
        current_dual_arm_robotstate_msg_.joint_state.velocity = plan_left.trajectory_.joint_trajectory.points[plan_left.trajectory_.joint_trajectory.points.size() - 1].velocities;
        left_.setStartStateToCurrentState();
        left_current_pose_ = left_.getCurrentPose(left_.getEndEffectorLink());
    }
    after_execute = ros::Time::now();
    execute_time = after_execute - before_execute;
    ROS_INFO(":::::: VALUES EVALUATION ::::::");
    ROS_INFO("Execute took: %f sec", execute_time.toSec());
    
    return true;
#ifdef OFFLINE

    double error = arms_.execute(plan);
    // sleep(2); // to be sure robot is at goal position
    left_current_pose_ = left_.getCurrentPose(left_.getEndEffectorLink());
    current_dual_arm_robotstate_msg_.is_diff = true;
    current_dual_arm_robotstate_msg_.joint_state.position = left_.getCurrentJointValues();
    current_dual_arm_robotstate_msg_.joint_state.name = left_.getJointNames();
    if (error == -1)
        return false;
    return true;
#endif
}

/* Move the robot to its home position defined in SRDF file */
bool DualArmRobot::moveHome()
{
    int try_count = 0;
    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;
    try_step = true;
    while (try_step && ros::ok())
    {
        arms_.setNamedTarget("arms_up");
        moveit::planning_interface::MoveGroupInterface::Plan arms_plan;
        error = arms_.plan(arms_plan);
        if (error.val != 1)
        {
            ROS_WARN("MoveIt!Error Code: %i", error.val);
            try_step = try_again_question();
            if (!try_step)
                return false;
        }
        else
        {
            dual_arm_toolbox::TrajectoryProcessor::clean(arms_plan.trajectory_);
            dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(arms_plan.trajectory_, 0.5);
            execute(arms_plan);
            try_step = false;
        }
    }
    return true;
}

/* Move the robot to its home position defined in SRDF file */
bool DualArmRobot::moveGraspPosition()
{
    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;
    try_step = true;
    while (try_step && ros::ok())
    {
        arms_.setNamedTarget("grasp_position");
        moveit::planning_interface::MoveGroupInterface::Plan arms_plan;
        error = arms_.plan(arms_plan);
        if (error.val != 1)
        {
            ROS_WARN("MoveIt!Error Code: %i", error.val);
            try_step = try_again_question();
            if (!try_step)
                return false;
        }
        else
        {
            dual_arm_toolbox::TrajectoryProcessor::clean(arms_plan.trajectory_);
            dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(arms_plan.trajectory_, 0.4);
            execute(arms_plan);
            try_step = false;
            /* Update the robot state */
            current_dual_arm_robotstate_msg_ = getCurrentRobotStateMsg();
        }
    }
    return true;
}
void DualArmRobot::publishPlanCartTrajectory(std::string endEffectorLink,
                                            const robot_state::JointModelGroup *joint_model_group,
                                            moveit_msgs::RobotTrajectory &trajectory)
{
    std::string plan_topic = "left/cart_pose_cmd";
    std::string joint_traj_point_topic = "left/joint_traj_point_cmd";
    std::size_t right_found = endEffectorLink.find("right");
    if (right_found!=std::string::npos){
        plan_topic = "right/cart_pose_cmd";
        joint_traj_point_topic = "right/joint_traj_point_cmd";
    } 
    ros::Publisher pub_cart_plan = nh_.advertise<geometry_msgs::PoseStamped>(plan_topic, 100);
    ros::Publisher pub_joint_traj_point_cmd_ = nh_.advertise<trajectory_msgs::JointTrajectoryPoint>(joint_traj_point_topic, 100);
    geometry_msgs::PoseStamped poseFK;
    poseFK.header.stamp = ros::Time::now();
    poseFK.header.frame_id = "world";
    ros::Rate loop_rate(100);
    std::vector<double> joint_values;
    int cmd_count = 0;
    ros::Duration time_step(0);
    for (unsigned int i = 0; i < trajectory.joint_trajectory.points.size(); i++)
    {
        joint_values.clear();
        auto point = trajectory.joint_trajectory.points[i];
        std::copy(point.positions.begin(), point.positions.end(),
                    std::back_inserter(joint_values));
        KDL::Frame frame_pose;
        kinematic_statePtr->setJointGroupPositions(joint_model_group, joint_values);
        const Eigen::Affine3d &Affine3d_pose = kinematic_statePtr->getGlobalLinkTransform(endEffectorLink);
        tf::transformEigenToKDL(Affine3d_pose, frame_pose);
        geometry_msgs::Pose pose_msg;
        dual_arm_toolbox::Transform::transformKDLtoPose(frame_pose, pose_msg);
        poseFK.pose = std::move(pose_msg);
        time_step =  point.time_from_start - time_step;
        poseFK.header.stamp = poseFK.header.stamp + time_step;
        pub_cart_plan.publish(poseFK);
        ++cmd_count;
        pub_joint_traj_point_cmd_.publish(point);
        time_step.sleep();
        time_step =  point.time_from_start;
    }
    ROS_INFO("Published %d msg for %ld points, %s", cmd_count, trajectory.joint_trajectory.points.size(), endEffectorLink.c_str());
}

void DualArmRobot::PrintPose(geometry_msgs::Pose &pose)
{
    ROS_INFO("Print Pose: x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f\n",
             pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

void DualArmRobot::publishOffsetPointState()
{
    ros::Publisher offset_pose_pub = nh_.advertise<geometry_msgs::Pose>("/offset_pose_state", 100);
    geometry_msgs::Pose offset_pose;
    ros::Rate rate(100);
    while (ros::ok())
    {
        KDL::Frame offset_frame = getCurrentOffset();  // in left EE coordinate system
        offset_pose.position.x = offset_frame.p.x();
        offset_pose.position.y = offset_frame.p.y();
        offset_pose.position.z = offset_frame.p.z();
        offset_frame.M.GetQuaternion(offset_pose.orientation.x, offset_pose.orientation.y, 
                                     offset_pose.orientation.z, offset_pose.orientation.w);
        offset_pose_pub.publish(offset_pose);
        ros::spinOnce();
        rate.sleep();
    }
}

void DualArmRobot::setConstraints()
{
    // setup constraints
    moveit_msgs::JointConstraint jcm;
    moveit_msgs::Constraints left_constraints;
    moveit_msgs::Constraints right_constraints;
    moveit_msgs::Constraints both_constraints;
    ROS_INFO("Start to set up the constraints...");
    // when placing box on top left can get blocked because wrist 1 reaches limit
    jcm.joint_name = "left_wrist_1_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 3.0;
    jcm.tolerance_below = 3.0;
    jcm.weight = 1;
    left_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    left_.setPathConstraints(left_constraints);

    jcm.joint_name = "left_shoulder_pan_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 3;
    jcm.tolerance_below = 3;
    jcm.weight = 1.0;
    left_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    left_.setPathConstraints(left_constraints);

    jcm.joint_name = "left_shoulder_lift_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 3;
    jcm.tolerance_below = 3;
    jcm.weight = 1.0;
    left_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    left_.setPathConstraints(left_constraints);

    // left can get blocked while placing without this constraint
    jcm.joint_name = "left_elbow_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 3;
    jcm.tolerance_below = 2;
    jcm.weight = 1;
    left_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    left_.setPathConstraints(left_constraints);

    jcm.joint_name = "left_wrist_3_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 3;
    jcm.tolerance_below = 1;
    jcm.weight = 1.0;
    left_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    left_.setPathConstraints(left_constraints);
    // left sometimes blocks itself when picking the box on top, this should solve the issue

    jcm.joint_name = "right_shoulder_pan_joint";
    jcm.position = 0.01;
    jcm.tolerance_above = 3;
    jcm.tolerance_below = 3;
    jcm.weight = 1.0;
    right_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    right_.setPathConstraints(right_constraints);

    jcm.joint_name = "right_shoulder_lift_joint";
    jcm.position = 0;
    jcm.tolerance_above = 6;
    jcm.tolerance_below = 3;
    jcm.weight = 1.0;
    right_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    right_.setPathConstraints(right_constraints);

    jcm.joint_name = "right_wrist_2_joint";
    jcm.position = 0;
    jcm.tolerance_above = 3;
    jcm.tolerance_below = 3;
    jcm.weight = 1.0;
    right_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    right_.setPathConstraints(right_constraints);

    jcm.joint_name = "right_wrist_3_joint";
    jcm.position = 0;
    jcm.tolerance_above = 3;
    jcm.tolerance_below = 3;
    jcm.weight = 1.0;
    right_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    right_.setPathConstraints(right_constraints);

    arms_.setPathConstraints(both_constraints);
}
bool DualArmRobot::MoveParallel(geometry_msgs::Pose &left_pose,
                                geometry_msgs::Pose &right_pose,
                                      double traj_scale)
{

    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;

    // move both at simultaneously
    moveit_msgs::RobotState robot_grasp_state_;
    robot_grasp_state_ = getCurrentRobotStateMsg();
    geometry_msgs::PoseStamped left_grasp_pose_ = left_.getCurrentPose(left_.getEndEffectorLink());
    geometry_msgs::PoseStamped right_grasp_pose_ = right_.getCurrentPose(right_.getEndEffectorLink());

    try_step = true;
    while (try_step && ros::ok())
    {
        arms_.setPoseTarget(left_pose, left_.getEndEffectorLink());
        arms_.setPoseTarget(right_pose, right_.getEndEffectorLink());
        moveit::planning_interface::MoveGroupInterface::Plan arms_plan;
        error = arms_.plan(arms_plan);
        if (error.val != 1)
        {
            ROS_WARN("MoveIt! error for MoveParallel Code: %i", error.val);
            try_step = try_again_question();
            if (!try_step)
                return false;
        }
        else
        {
            dual_arm_toolbox::TrajectoryProcessor::clean(arms_plan.trajectory_);
            dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(arms_plan.trajectory_, traj_scale);
            execute(arms_plan);
            try_step = false;
            /* Update the robot state */
            current_dual_arm_robotstate_msg_ = getCurrentRobotStateMsg();
        }
    }
    return true;

}

bool DualArmRobot::get_rotation_matrix(Matrix6d &rotation_matrix,
						 tf::TransformListener &listener,
						 std::string from_frame,
						 std::string to_frame)
{
	tf::StampedTransform transform;
	Matrix3d rotation_from_to;
	try
	{
		listener.lookupTransform(from_frame, to_frame, ros::Time(0), transform);
		tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
		rotation_matrix.setZero();
		rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
		rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
	}
	catch (tf::TransformException ex)
	{
		rotation_matrix.setZero();
		ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame);
		return false;
	}

	return true;
}

bool DualArmRobot::executeAC(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/robot/limb/left/follow_joint_trajectory", true);
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_action", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(1.0);

  ac.sendGoal(goal);
  //std::cout << "siz is: " << goal.trajectory.points.size()-1 << std::endl;
  if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start + ros::Duration(5)))
  {
    ROS_INFO("Action server reported successful execution");
    return true;
  } else {
    ROS_WARN("Action server could not execute trajectory");
    return false;
  }
}
