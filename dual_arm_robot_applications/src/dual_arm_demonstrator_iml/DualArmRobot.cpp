//
// Created by Chunting  on 20.10.16.
//

#include <dual_arm_demonstrator_iml/SceneManager.h>
#include "dual_arm_demonstrator_iml/DualArmRobot.h"

using namespace dual_arm_demonstrator_iml;

DualArmRobot::DualArmRobot(ros::NodeHandle& nh) :
        left_("left_manipulator"),
        right_("right_manipulator"),
        arms_("arms"),
        nh_(nh) {
    // MoveIt! Setup
    //left_.setPlanningTime(40);
    // Spceciy the maximum amount of time to use when planning
    left_.setPlanningTime(5);
    // Set the number of times the motion plan is to be computed, the default value is 1
    left_.setNumPlanningAttempts(25);
    //right_.setPlanningTime(40);
    right_.setPlanningTime(5);
    right_.setNumPlanningAttempts(25);

    // setup planner
    // Specify a planner to be used for further planning
    left_.setPlannerId("RRTConnectkConfigDefault");
    right_.setPlannerId("RRTConnectkConfigDefault");
    // Allow replanning
    left_.allowReplanning(true);
    right_.allowReplanning(true);

    // planning scene monitor
    // Subscribes to the topic planning_scene
    planning_scene_monitor_ = boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

    // initialize ur5 specific variables
    // left_.getEndEffectorLink() get current end-effector link
    // frame_id:  /world
    left_current_pose_ = left_.getCurrentPose(left_.getEndEffectorLink());
    left_current_robot_state_ = getCurrentRobotStateMsg();

    // left_joint_values = getJointAngles("left_manipulator");
    ROS_INFO("left_current_pose_ frame_id: %s, x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f", 
        left_current_pose_.header.frame_id.c_str()
        ,left_current_pose_.pose.position.x
        ,left_current_pose_.pose.position.y
        ,left_current_pose_.pose.position.z
        ,left_current_pose_.pose.orientation.x
        ,left_current_pose_.pose.orientation.y
        ,left_current_pose_.pose.orientation.z
        ,left_current_pose_.pose.orientation.w);

    right_current_pose_ = right_.getCurrentPose(right_.getEndEffectorLink());
    ROS_INFO("right_current_pose_ frame_id: %s, x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f", 
        right_current_pose_.header.frame_id.c_str()
        ,right_current_pose_.pose.position.x
        ,right_current_pose_.pose.position.y
        ,right_current_pose_.pose.position.z
        ,right_current_pose_.pose.orientation.x
        ,right_current_pose_.pose.orientation.y
        ,right_current_pose_.pose.orientation.z
        ,right_current_pose_.pose.orientation.w);
    ROS_INFO("\nPublish the right arm joint state\n");
    ROS_INFO("Reference Frame: %s", right_.getPlanningFrame().c_str());
    ROS_INFO("Reference Frame: %s", right_.getEndEffectorLink().c_str());
    std::vector<double> rpy = right_.getCurrentRPY();
    ROS_INFO("RPY: R=%f, P=%f, Y=%f", rpy[0], rpy[1], rpy[2]);
    std::vector<std::string> rightJointNames = right_.getJoints();
    std::vector<double> rightJointValues = right_.getCurrentJointValues();
    for(std::size_t i=0; i<rightJointNames.size(); i++){
        ROS_INFO("Joint %s: %f", rightJointNames[i].c_str(), radianToDegree(rightJointValues[i]));
    }


    ROS_INFO("\nPublish the left arm joint state\n");
        
    ROS_INFO("Reference Frame: %s", left_.getPlanningFrame().c_str());
    ROS_INFO("Reference Frame: %s", left_.getEndEffectorLink().c_str());
    std::vector<std::string> leftJointNames = left_.getJoints();
    std::vector<double> leftJointValues = left_.getCurrentJointValues();
    for(std::size_t i=0; i<leftJointNames.size(); i++){
        ROS_INFO("Joint %s: %f", leftJointNames[i].c_str(), radianToDegree(leftJointValues[i]));
    }
    ROS_INFO("\nPublish the both arms joint state\n");
    ROS_INFO("Reference Frame: %s", arms_.getPlanningFrame().c_str());
    // ROS_INFO("Reference Frame: %s", arms_.getEndEffectorLink().c_str());
    std::vector<std::string> armsJointNames = arms_.getJoints();
    std::vector<double> armsJointValues = arms_.getCurrentJointValues();
    for(std::size_t i=0; i<armsJointNames.size(); i++){
        ROS_INFO("Joint %s: %f", armsJointNames[i].c_str(), armsJointValues[i]);
    }


    ROS_INFO("Planning test");
    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;



    // move both arms
    try_step = true;
    while (try_step && ros::ok()) {
        // geometry_msgs::PoseStamped left_pose;
        //left_pose.header.frame_id = "/world";
        left_current_pose_.pose.position.x = left_current_pose_.pose.position.x-0.1;
        // left_pose.pose.position.y = -0.116;
        // left_pose.pose.position.z = 0.606; //0.4+0.1
        // left_pose.pose.orientation.x = 0.429;
        // left_pose.pose.orientation.y = 0.480;
        // left_pose.pose.orientation.z = 0.601;
        // left_pose.pose.orientation.w = 0.473;
        // KDL::Rotation left_rot;  // generated to easily assign quaternion of pose
        // left_rot.DoRotY(3.14 / 2);
        // left_rot.GetQuaternion(left_pose.pose.orientation.x, left_pose.pose.orientation.y, left_pose.pose.orientation.z,
        //                       left_pose.pose.orientation.w);

        //geometry_msgs::PoseStamped right_pose;
        //right_pose.header.frame_id = "/world";
        right_current_pose_.pose.position.x = right_current_pose_.pose.position.x+0.1;;
        // right_pose.pose.position.y = 0.085;
        // right_pose.pose.position.z = 0.609;
        // right_pose.pose.orientation.x = -0.001;
        // right_pose.pose.orientation.y = -0.023;
        // right_pose.pose.orientation.z = -0.679;
        // right_pose.pose.orientation.w = 0.734;
        KDL::Rotation right_rot;  // generated to easily assign quaternion of pose
        //right_rot.DoRotY(3.14 / 2);
        //right_rot.DoRotX(3.14);
        // right_rot.GetQuaternion(right_pose.pose.orientation.x, right_pose.pose.orientation.y, right_pose.pose.orientation.z,
        //                        right_pose.pose.orientation.w);

        arms_.setStartState(left_current_robot_state_);
        arms_.setPoseTarget(left_current_pose_, left_.getEndEffectorLink());
        arms_.setPoseTarget(right_current_pose_, right_.getEndEffectorLink());
        // The representation of a motion plan (as ROS messages), it's a structure.
        moveit::planning_interface::MoveGroup::Plan arms_plan; 
        // arms_ is a class of MoveGroup
        ROS_INFO("Left arm target pose: %s  %f  %f  %f", 
            left_current_pose_.header.frame_id.c_str(), 
            left_current_pose_.pose.position.x,
            left_current_pose_.pose.position.y,
            left_current_pose_.pose.position.z);
         ROS_INFO("\nRight arm target pose: %s  %f  %f  %f", 
            right_current_pose_.header.frame_id.c_str(), 
            right_current_pose_.pose.position.x,
            right_current_pose_.pose.position.y,
            right_current_pose_.pose.position.z);

        error = arms_.plan(arms_plan);
          // visualize plan
        dual_arm_toolbox::TrajectoryProcessor::visualizePlan(arms_plan, 5);
        if (error.val != 1) {
            ROS_WARN("MoveIt!Error Code: %i", error.val);
            try_step = try_again_question();
            
        } else {
            ROS_INFO("Moving two arms into grasp position");
            dual_arm_toolbox::TrajectoryProcessor::clean(arms_plan.trajectory_);
            dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(arms_plan.trajectory_, 0.4);
            execute(arms_plan);
            try_step = false;
        }
    }

  
    


  // END_TUTORIAL

    // Controller Interface
    /*
#ifdef SIMULATION
    left_controller_ = "fake_left_manipulator_controller";
    right_controller_ = "fake_right_manipulator_controller";
#endif
#ifndef SIMULATION*/
    left_controller_ = "left/left_vel_based_pos_traj_controller";
    right_controller_ = "right/right_vel_based_pos_traj_controller";
//#endif
}

robot_state::RobotState DualArmRobot::getCurrentRobotState(){
    // Request planning scene state using a service call (service name)
    planning_scene_monitor_->requestPlanningSceneState("get_planning_scene");

    // This is a convenience class for obtaining access to an instance of a l

    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);

    ps->getCurrentStateNonConst().update();
    // Definition of a kinematic state (both joint and link)
    robot_state::RobotState current_state = ps->getCurrentState();
    return current_state;
}
// It's a class object
moveit_msgs::RobotState DualArmRobot::getCurrentRobotStateMsg() {
    // # This message contains information about the robot state, i.e. the po

    // http://docs.ros.org/melodic/api/moveit_msgs/html/msg/RobotState.html
    moveit_msgs::RobotState current_state_msg;
    // Convert a MoveIt! robot state to a robot state message. 
    moveit::core::robotStateToRobotStateMsg(getCurrentRobotState(), current_state_msg);
    return current_state_msg;
}
// returns the Offset-Vector between both end effectors
// KDL::Frame
// represents a frame transformation in 3D space (rotation + translation) 
// if V2 = Frame*V1 (V2 expressed in frame A, V1 expressed in frame B) then V2 = Frame.M*V1+Frame.p
// Frame.M contains columns that represent the axes of frame B wrt frame A 
// Frame.p contains the origin of frame B expressed in frame A. 
KDL::Frame DualArmRobot::getCurrentOffset(){
    geometry_msgs::PoseStamped left_pose = left_.getCurrentPose(left_.getEndEffectorLink());
    geometry_msgs::PoseStamped right_pose = right_.getCurrentPose(right_.getEndEffectorLink());
    KDL::Frame left_frame;
    KDL::Frame right_frame;
    dual_arm_toolbox::Transform::transformPoseToKDL(left_pose.pose, left_frame);
    dual_arm_toolbox::Transform::transformPoseToKDL(right_pose.pose, right_frame);
    KDL::Frame offset;
    offset = left_frame.Inverse() * right_frame;
    return offset;
}
 // calculates a trajectory for both arms based on the trajectory of one arm
 // moveit_msgs::RobotTrajectory is a class object that contains two subclass, JointTrajectory and multi_dof_joint_trajectory
bool DualArmRobot::adaptTrajectory(moveit_msgs::RobotTrajectory left_trajectory, 
                                    KDL::Frame offset, 
                                    moveit_msgs::RobotTrajectory& both_arms_trajectory, 
                                    double jump_threshold){
    // setup planning scene
    // Look up the robot description on the ROS parameter server and construct a RobotModel to use
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planningScene(kinematic_model);

    // setup JointModelGroup
    //ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    // Construct a RobotState that maintains the configuration of the robot.
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    // Set all the joints to their default values.
    // Set all joints to their default positions.
    // The default position is 0, or if that is not within bounds then half way between min and max bound. 
    kinematic_state->setToDefaultValues();
    // Represents the robot model for a particular group.
    const robot_state::JointModelGroup* left_joint_model_group = kinematic_model->getJointModelGroup("left_manipulator");
    const robot_state::JointModelGroup* right_joint_model_group = kinematic_model->getJointModelGroup("right_manipulator");
    // const std::vector<std::string>& left_joint_names = left_joint_model_group->getJointModelNames();
    // const std::vector<std::string>& right_joint_names = right_joint_model_group->getJointModelNames();


    // setup both_arms_trajectory message
    both_arms_trajectory = left_trajectory;
    for (unsigned int j=0; j < right_joint_model_group->getActiveJointModelNames().size(); j++){
        both_arms_trajectory.joint_trajectory.joint_names.push_back(right_joint_model_group->getActiveJointModelNames()[j]);
    }

    // ik service client setup
    // moveit_msgs::GetPositionIK # A service call to carry out an inverse kinematics computation
    ros::ServiceClient ik_client = nh_.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    // A service call to carry out an inverse kinematics computation
    moveit_msgs::GetPositionIK ik_msg;

    // current state for initial seed
    //moveit_msgs::RobotState robot_state;
    //moveit::core::robotStateToRobotStateMsg(*arms_.getCurrentState(), robot_state);
    //ik_msg.request.ik_request.robot_state = robot_state;
    ik_msg.request.ik_request.robot_state = getCurrentRobotStateMsg();
    ik_msg.request.ik_request.attempts = 5;
    ik_msg.request.ik_request.avoid_collisions = true;
    ik_msg.request.ik_request.group_name = right_.getName();
    ik_msg.request.ik_request.timeout = ros::Duration(15);

    // computing values for trajectory
    for (unsigned int i = 0; i < left_trajectory.joint_trajectory.points.size(); i++){
        std::vector<double> left_joint_values;
        left_joint_values.clear();
        for (unsigned int a = 0; a < left_trajectory.joint_trajectory.points[i].positions.size(); a++){
            left_joint_values.push_back(left_trajectory.joint_trajectory.points[i].positions[a]);
        }
        // fk -> pos left
        KDL::Frame frame_pose_left;
        kinematic_state->setJointGroupPositions(left_joint_model_group, left_joint_values);
        const Eigen::Affine3d &end_effector_pose_left = kinematic_state->getGlobalLinkTransform(left_.getEndEffectorLink());
        tf::transformEigenToKDL(end_effector_pose_left, frame_pose_left);

        // compute pos right
        KDL::Frame frame_pose_right = frame_pose_left*offset;

        // ik msg for request for right
        ik_msg.request.ik_request.pose_stamped.header.frame_id = "world";
        ik_msg.request.ik_request.pose_stamped.pose.position.x = frame_pose_right.p.x();
        ik_msg.request.ik_request.pose_stamped.pose.position.y = frame_pose_right.p.y();
        ik_msg.request.ik_request.pose_stamped.pose.position.z = frame_pose_right.p.z();
        // Get the quaternion of this matrix(frame_pose_right) 
        frame_pose_right.M.GetQuaternion(
                ik_msg.request.ik_request.pose_stamped.pose.orientation.x,
                ik_msg.request.ik_request.pose_stamped.pose.orientation.y,
                ik_msg.request.ik_request.pose_stamped.pose.orientation.z,
                ik_msg.request.ik_request.pose_stamped.pose.orientation.w
        );

        // get ik solution
        bool try_again;
        double try_count = 0;
        do {
            ik_client.call(ik_msg.request, ik_msg.response);
            if (ik_msg.response.error_code.val != 1) {
                ROS_WARN("ik request error");
                return false;
            }
            // try again if jump is too huge
            try_again = false;
            for (unsigned int a = 0; a < ik_msg.response.solution.joint_state.position.size(); a++){
                try_again = try_again || (std::abs(ik_msg.response.solution.joint_state.position[a] - ik_msg.request.ik_request.robot_state.joint_state.position[a]) > jump_threshold);
            }
            if (try_again) {
                ROS_INFO("Ik: jump detected. One value deviates more than %f. Trying again", jump_threshold);
                try_count++;
                if (try_count > 10){
                    ROS_WARN("could not find solution without jump");
                    return false;
                }
            }
        } while (try_again);

        // write results into trajectory msg
        for (unsigned int j=0; j < ik_msg.response.solution.joint_state.position.size(); j++){
            both_arms_trajectory.joint_trajectory.points[i].positions.push_back(ik_msg.response.solution.joint_state.position[j]);
        }

        // use result as seed for next calculation
        ik_msg.request.ik_request.robot_state = ik_msg.response.solution;
    }

    // execution speed
    dual_arm_toolbox::TrajectoryProcessor::computeTimeFromStart(both_arms_trajectory, 0.4);

    // compute velocities for trajectory
    return dual_arm_toolbox::TrajectoryProcessor::computeVelocities(both_arms_trajectory, arms_);
}

bool DualArmRobot::switch_controller(std::string stop_name, std::string start_name, std::string ur_namespace) {
    ROS_INFO("Switching controllers");
#ifndef OFFLINE
    // setup
    ros::ServiceClient srv_switch_controller = nh_.serviceClient<controller_manager_msgs::SwitchController>(ur_namespace+"/controller_manager/switch_controller");
    controller_manager_msgs::SwitchController switchController;
    switchController.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
    sleep(2);

    // stop
    switchController.request.stop_controllers.push_back(stop_name);
    bool success_stop = srv_switch_controller.call(switchController);
    ROS_INFO("Stopping controller %s",success_stop?"SUCCEDED":"FAILED");
    if (!success_stop) return false;

    // clear
    switchController.request.stop_controllers.clear();

    // start admittance controller
    switchController.request.BEST_EFFORT;
    switchController.request.start_controllers.push_back(start_name);
    bool success_start = srv_switch_controller.call(switchController);
    ROS_INFO("Starting controller %s",success_start?"SUCCEDED":"FAILED");
    switchController.request.start_controllers.clear();

    // Switch controller in moveit-interface
    if (success_start){
        if (left_controller_.compare(0,ur_namespace.size(),ur_namespace)==0){
            left_controller_ = ur_namespace+"/"+start_name;
        }
        if (right_controller_.compare(0,ur_namespace.size(),ur_namespace)==0){
            right_controller_ = ur_namespace+"/"+start_name;
        }
    }

    return success_start;
#endif
#ifdef OFFLINE
    return true;
#endif
}

bool DualArmRobot::graspMove(double distance, bool avoid_collisions, bool use_left, bool use_right) {
    bool try_step;
    double fraction;

    // move closer with right and ur5 by using cartesian path
    if (distance > 0) ROS_INFO("Moving towards object");
    if (distance < 0) ROS_INFO("Moving away from object");

    // ur5
    if (use_left) try_step = true;
    while (try_step && ros::ok()) {
        left_.setStartState(left_current_robot_state_);
        std::vector<geometry_msgs::Pose> left_waypoints;
        geometry_msgs::Pose left_waypoint = left_current_pose_.pose;
        left_waypoints.push_back(left_waypoint);

        // transform distance vector
        KDL::Vector left_vec_d;  // distance vector
        KDL::Frame left_p_eef;   // endeffector frame
        dual_arm_toolbox::Transform::transformPoseToKDL(left_waypoint, left_p_eef);
        left_vec_d.x(0);
        left_vec_d.y(0);
        left_vec_d.z(distance);
        left_vec_d = left_p_eef.M * left_vec_d;     // Rotate distance vector

        // calculate waypoint
        left_waypoint.position.x = left_waypoint.position.x + left_vec_d.x();
        left_waypoint.position.y = left_waypoint.position.y + left_vec_d.y();
        left_waypoint.position.z = left_waypoint.position.z + left_vec_d.z();

        left_waypoints.push_back(left_waypoint);
        moveit_msgs::RobotTrajectory left_trajectory;
        fraction = left_.computeCartesianPath(left_waypoints, 0.001, 0.0, left_trajectory, avoid_collisions);
        if (fraction < 0.9) {
            ROS_WARN("Left arm cartesian path. (%.2f%% achieved)", fraction * 100.0);
            try_step = try_again_question();
            if (!try_step) return false;
        }
        else {
            dual_arm_toolbox::TrajectoryProcessor::clean(left_trajectory);
            moveit::planning_interface::MoveGroup::Plan left_plan;
            left_plan.trajectory_ = left_trajectory;
            execute(left_plan);
            try_step = false;
        }
    }

    // right
    if (use_right) try_step = true;
    while (try_step && ros::ok()) {
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
        moveit_msgs::RobotTrajectory right_trajectory;
        fraction = right_.computeCartesianPath(right_waypoints, 0.001, 0.0, right_trajectory, avoid_collisions);
        if (fraction < 0.9) {
            ROS_WARN("Right arm cartesian path. (%.2f%% achieved)", fraction * 100.0);
            try_step = try_again_question();
            if (!try_step) return false;
        } else {
            moveit::planning_interface::MoveGroup::Plan right_plan;
            dual_arm_toolbox::TrajectoryProcessor::clean(right_trajectory);
            right_plan.trajectory_ = right_trajectory;
            execute(right_plan);
            try_step = false;
        }
    }

    return true;
}

bool DualArmRobot::try_again_question() {
    // ask to try again
    /*
    std::cout << "Try this step again? (y/n) ";
    char response;
    std::cin >> response;
    if (response == 'n') return false;
    else return true;*/

    // automatic retry
    if (ros::ok()){
        ROS_WARN("Trying this step again. Press Ctr+C to abort.");
        return true;
    }
    return false;
}

bool DualArmRobot::pickBox(std::string object_id , geometry_msgs::Vector3Stamped lift_direction) {
    ROS_INFO("Starting Pick Box sequence, in the frame of %s", object_id.c_str());
    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;

    // move left arm
    
    // try_step = true;
    // while (try_step && ros::ok()) {
    //     geometry_msgs::PoseStamped left_pose;
    //     left_pose.header.frame_id = "table";
    //     left_pose.pose.position.x = 0.176;
    //     left_pose.pose.position.y = -0.275;
    //     left_pose.pose.position.z = 0.642+0.1; //0.4+0.1
    //     left_pose.pose.orientation.x = 0.657;
    //     left_pose.pose.orientation.y = 0.753;
    //     left_pose.pose.orientation.z = -0.004;
    //     left_pose.pose.orientation.w = 0.043;
    //     // KDL::Rotation left_rot;  // generated to easily assign quaternion of pose
    //     // left_rot.DoRotY(3.14 / 2);
    //     // left_rot.GetQuaternion(left_pose.pose.orientation.x, left_pose.pose.orientation.y, left_pose.pose.orientation.z,
    //     //                       left_pose.pose.orientation.w);
    //     left_.setStartState(left_current_robot_state_);
    //     left_.setPoseTarget(left_pose);
    //     moveit::planning_interface::MoveGroup::Plan left_plan;
    //     error = left_.plan(left_plan);
    //     if (error.val != 1) {
    //         ROS_WARN("MoveIt!Error Code: %i", error.val);
    //         try_step = try_again_question();
    //         if (!try_step) return false;
    //     } else {
    //         ROS_INFO("Moving left arm into grasp position");
    //         dual_arm_toolbox::TrajectoryProcessor::clean(left_plan.trajectory_);
    //         dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(left_plan.trajectory_, 0.4);
    //         execute(left_plan);
    //         try_step = false;
    //     }
    // }

    // move right arm
    /*
    try_step = true;
    while (try_step && ros::ok()) {
        geometry_msgs::PoseStamped right_pose;
        right_pose.header.frame_id = "table";
        right_pose.pose.position.x = 0.148;
        right_pose.pose.position.y = 0.025;
        right_pose.pose.position.z = 0.688+0.1;
        right_pose.pose.orientation.x = 0.530;
        right_pose.pose.orientation.x = -0.467;
        right_pose.pose.orientation.x = -0.464;
        right_pose.pose.orientation.x = 0.534;
        // KDL::Rotation right_rot;  // generated to easily assign quaternion of pose
        // right_rot.DoRotY(3.14 / 2);
        // right_rot.DoRotX(3.14);
        // right_rot.GetQuaternion(right_pose.pose.orientation.x, right_pose.pose.orientation.y, right_pose.pose.orientation.z,
        //                        right_pose.pose.orientation.w);
        right_.setPoseTarget(right_pose);

        moveit::planning_interface::MoveGroup::Plan right_plan;
        error = right_.plan(right_plan);
        if (error.val != 1) {
            ROS_WARN("MoveIt!Error Code: %i", error.val);
            try_step = try_again_question();
            if (!try_step) return false;
        } else {
            ROS_INFO("Moving right into grasp position");
            dual_arm_toolbox::TrajectoryProcessor::clean(right_plan.trajectory_);
            dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(right_plan.trajectory_, 0.4);
            execute(right_plan);
            try_step = false;
        }
    }
    */
    
    // move both arms
    try_step = true;
    while (try_step && ros::ok()) {
        geometry_msgs::PoseStamped left_pose;
        left_pose.header.frame_id = "table";
        left_pose.pose.position.x = 0.303;
        left_pose.pose.position.y = -0.116;
        left_pose.pose.position.z = 0.606; //0.4+0.1
        left_pose.pose.orientation.x = 0.429;
        left_pose.pose.orientation.y = 0.480;
        left_pose.pose.orientation.z = 0.601;
        left_pose.pose.orientation.w = 0.473;
        // KDL::Rotation left_rot;  // generated to easily assign quaternion of pose
        // left_rot.DoRotY(3.14 / 2);
        // left_rot.GetQuaternion(left_pose.pose.orientation.x, left_pose.pose.orientation.y, left_pose.pose.orientation.z,
        //                       left_pose.pose.orientation.w);

        geometry_msgs::PoseStamped right_pose;
        right_pose.header.frame_id = "table";
        right_pose.pose.position.x = 0.266;
        right_pose.pose.position.y = 0.085;
        right_pose.pose.position.z = 0.609;
        right_pose.pose.orientation.x = -0.001;
        right_pose.pose.orientation.y = -0.023;
        right_pose.pose.orientation.z = -0.679;
        right_pose.pose.orientation.w = 0.734;
        KDL::Rotation right_rot;  // generated to easily assign quaternion of pose
        //right_rot.DoRotY(3.14 / 2);
        //right_rot.DoRotX(3.14);
        // right_rot.GetQuaternion(right_pose.pose.orientation.x, right_pose.pose.orientation.y, right_pose.pose.orientation.z,
        //                        right_pose.pose.orientation.w);

        arms_.setStartState(left_current_robot_state_);
        arms_.setPoseTarget(left_pose, left_.getEndEffectorLink());
        arms_.setPoseTarget(right_pose, right_.getEndEffectorLink());
        // The representation of a motion plan (as ROS messages), it's a structure.
        moveit::planning_interface::MoveGroup::Plan arms_plan; 
        // arms_ is a class of MoveGroup
        ROS_INFO("Left arm target pose: %s  %f  %f  %f", 
            left_pose.header.frame_id.c_str(), 
            left_pose.pose.position.x,
            left_pose.pose.position.y,
            left_pose.pose.position.z);
         ROS_INFO("\nRight arm target pose: %s  %f  %f  %f", 
            right_pose.header.frame_id.c_str(), 
            right_pose.pose.position.x,
            right_pose.pose.position.y,
            right_pose.pose.position.z);

        error = arms_.plan(arms_plan);
        if (error.val != 1) {
            ROS_WARN("MoveIt!Error Code: %i", error.val);
            try_step = try_again_question();
            if (!try_step) return false;
        } else {
            ROS_INFO("Moving two arms into grasp position");
            dual_arm_toolbox::TrajectoryProcessor::clean(arms_plan.trajectory_);
            dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(arms_plan.trajectory_, 0.4);
            execute(arms_plan);
            try_step = false;
        }
    }


    // Grasp
    // move closer with right and left arm by using cartesian path
    //if (!graspMove(0.075)) {
    if (!graspMove(0.075)) {
        ROS_WARN("failed moving closer to object");
        return false;
    };

    // attach object to ur and update State Msg
    left_.attachObject(object_id, left_.getEndEffectorLink());
    left_current_robot_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // compute cartesian Path for left arm
    left_.setStartState(left_current_robot_state_);
    moveit_msgs::RobotTrajectory left_trajectory;
    try_step = true;
    while (try_step && ros::ok()) {
        left_.setPoseReferenceFrame(lift_direction.header.frame_id);
        allowedArmCollision(true, object_id);
        std::vector<geometry_msgs::Pose> left_waypoints;
        geometry_msgs::Pose left_waypoint = left_.getCurrentPose(left_.getEndEffectorLink()).pose;
        left_waypoints.push_back(left_waypoint);
        left_waypoint.position.x = left_waypoint.position.x + lift_direction.vector.x;
        left_waypoint.position.y = left_waypoint.position.y + lift_direction.vector.y;
        left_waypoint.position.z = left_waypoint.position.z + lift_direction.vector.z;
        //left_current_pose_temp_.pose = left_waypoint; // TODO: find better solution
        //left_current_pose_temp_.header.frame_id = "world";
        left_waypoints.push_back(left_waypoint);
        double fraction = left_.computeCartesianPath(left_waypoints, 0.001, 0.0, left_trajectory, false);
        if (fraction < 1) {
            ROS_WARN("Left arm cartesian path. (%.2f%% achieved)", fraction * 100.0);
            ROS_WARN("Fraction is less than 100%%. Insufficient for dual-arm configuration. Pick can not be executed");
            try_step = try_again_question();
            if (!try_step){
                allowedArmCollision(false,object_id);
                return false;
            }
        }
        else try_step = false;
    }

    // get both arms trajectory
    arms_offset_ = getCurrentOffset();
    moveit_msgs::RobotTrajectory both_arms_trajectory;
    if (adaptTrajectory(left_trajectory, arms_offset_, both_arms_trajectory)) 
        ROS_INFO("successfully calculated trajectory for both arms");
    else {
        ROS_WARN("Problem adapting trajectory");
        allowedArmCollision(false,object_id);
        return false;
    }
    dual_arm_toolbox::TrajectoryProcessor::clean(both_arms_trajectory);
    allowedArmCollision(false,object_id);

    // get plan from trajectory
    moveit::planning_interface::MoveGroup::Plan both_arms_plan;
    both_arms_plan.trajectory_ = both_arms_trajectory;
    both_arms_plan.start_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // Grasp by switching controller and wait for contact while visualizing plan
    if (!switch_controller("left_vel_based_pos_traj_controller", "left_vel_based_admittance_traj_controller", "ur5"))
        ROS_WARN("failed switching controller");

    // visualize plan
    dual_arm_toolbox::TrajectoryProcessor::visualizePlan(both_arms_plan, 5);

    ROS_INFO("executing plan");
    execute(both_arms_plan);
    
    return true;
}

bool DualArmRobot::linearMoveParallel(geometry_msgs::Vector3Stamped direction, std::string object_id, double traj_scale, bool avoid_collisions) {
    // compute cartesian Path for ur5
    moveit_msgs::RobotTrajectory left_trajectory;
    bool try_step = true;
    while (try_step && ros::ok()) {
        left_.setPoseReferenceFrame(direction.header.frame_id);
        left_.setStartState(left_current_robot_state_);
        allowedArmCollision(true, object_id);
        std::vector<geometry_msgs::Pose> left_waypoints;
        geometry_msgs::Pose left_waypoint = left_current_pose_.pose;
        left_waypoints.push_back(left_waypoint);
        left_waypoint.position.x = left_waypoint.position.x + direction.vector.x;
        left_waypoint.position.y = left_waypoint.position.y + direction.vector.y;
        left_waypoint.position.z = left_waypoint.position.z + direction.vector.z;
        left_waypoints.push_back(left_waypoint);
        double fraction = left_.computeCartesianPath(left_waypoints, 0.001, 0.0, left_trajectory, false);
        if (fraction < 1) {
            ROS_INFO("Left arm cartesian path. (%.2f%% achieved)", fraction * 100.0);
            ROS_INFO("Fraction is less than 100%%. Insufficient for dual-arm configuration. Linear parallel move can not be executed");
            try_step = try_again_question();
            if (!try_step){
                allowedArmCollision(false,object_id);
                return false;
            }
        }
        else try_step = false;
    }

    dual_arm_toolbox::TrajectoryProcessor::clean(left_trajectory);
    // get both arms trajectory
    arms_offset_ = getCurrentOffset();
    moveit_msgs::RobotTrajectory both_arms_trajectory;
    if (adaptTrajectory(left_trajectory, arms_offset_, both_arms_trajectory)) ROS_INFO("successfully calculated trajectory for both arms");
    else {
        ROS_WARN("Problem adapting trajectory");
        allowedArmCollision(false,object_id);
        return false;
    }
    dual_arm_toolbox::TrajectoryProcessor::clean(both_arms_trajectory);
    allowedArmCollision(false,object_id);

    // scale trajectory
    dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(both_arms_trajectory, traj_scale);

    // get plan from trajectory
    moveit::planning_interface::MoveGroup::Plan both_arms_plan;
    both_arms_plan.trajectory_ = both_arms_trajectory;
    both_arms_plan.start_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // visualize plan
//    dual_arm_toolbox::TrajectoryProcessor::visualizePlan(both_arms_plan, 0.1);

    ROS_INFO("executing plan");
    execute(both_arms_plan);
    return true;
}

bool DualArmRobot::placeBox(std::string object_id, geometry_msgs::PoseStamped box_place_pose,
                            geometry_msgs::Vector3 close_direction) {
    ROS_INFO("Starting Place Box sequence");
    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;

    // calculate targets
    geometry_msgs::PoseStamped left_target;
    left_target = box_place_pose;
    KDL::Rotation left_rot;  // generated to easily assign quaternion of pose
    left_rot.DoRotY(3.14 / 2);
    left_rot.GetQuaternion(left_target.pose.orientation.x, left_target.pose.orientation.y, left_target.pose.orientation.z,
                          left_target.pose.orientation.w);
    KDL::Vector shift;
    SceneManager sceneManager(nh_);
    shift.z(-sceneManager.box_.dimensions[0]/2);
    shift = left_rot*shift;
    left_target.pose.position.x = box_place_pose.pose.position.x - close_direction.x + shift.x();
    left_target.pose.position.y = box_place_pose.pose.position.y - close_direction.y + shift.y();
    left_target.pose.position.z = box_place_pose.pose.position.z - close_direction.z + shift.z();

    // target position before placing
    moveObject(object_id, left_target);

    // compute cartesian Path for ur5
    moveit_msgs::RobotTrajectory left_trajectory_2;
    try_step = true;
    while (try_step && ros::ok()) {
        allowedArmCollision(true, object_id);
        std::vector<geometry_msgs::Pose> left_waypoints;
        left_.setStartState(left_current_robot_state_);
        geometry_msgs::Pose left_waypoint = left_current_pose_.pose;
        left_waypoints.push_back(left_waypoint);
        left_waypoint.position.x = left_waypoint.position.x + close_direction.x;
        left_waypoint.position.y = left_waypoint.position.y + close_direction.y;
        left_waypoint.position.z = left_waypoint.position.z + close_direction.z;
        left_waypoints.push_back(left_waypoint);
        double fraction = left_.computeCartesianPath(left_waypoints, 0.001, 0.0, left_trajectory_2, false);
        if (fraction < 1) {
            ROS_INFO("Left arm cartesian path. (%.2f%% achieved)", fraction * 100.0);
            ROS_INFO("Fraction is less than 100%%. Insufficient for dual-arm configuration. Pick can not be executed");
            try_step = try_again_question();
            if (!try_step){
                allowedArmCollision(false,object_id);
                return false;
            }
        }
        else try_step = false;
    }

    // get both arms trajectory
    moveit_msgs::RobotTrajectory both_arms_trajectory_2;
    if (adaptTrajectory(left_trajectory_2, arms_offset_, both_arms_trajectory_2, 1.0)) ROS_INFO("successfully calculated trajectory for both arms");
    else {
        ROS_WARN("Problem adapting trajectory");
        return false;
    }
    dual_arm_toolbox::TrajectoryProcessor::clean(both_arms_trajectory_2);
    allowedArmCollision(false,object_id);

    // get plan from trajectory
    moveit::planning_interface::MoveGroup::Plan both_arms_plan;
    both_arms_plan.trajectory_ = both_arms_trajectory_2;
//    dual_arm_toolbox::TrajectoryProcessor::visualizePlan(both_arms_plan, 5);
    both_arms_plan.start_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // execute
    execute(both_arms_plan);

    // release clamp
    // switch controller
    if (!switch_controller("left_vel_based_admittance_traj_controller", "left_vel_based_pos_traj_controller", "ur5"))
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
    left_current_robot_state_ = getCurrentRobotStateMsg();
    linearMove(correct_vec, false, true, false);

    // detach object
    left_.detachObject(object_id);
    left_current_robot_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // un-grasp
    graspMove(-0.08, false);

    return true;
}

bool DualArmRobot::pushPlaceBox(std::string object_id, geometry_msgs::PoseStamped box_pose, geometry_msgs::Vector3 direction) {
    ROS_INFO("Starting Push Place Box sequence");
    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;
    dual_arm_demonstrator_iml::SceneManager sceneManager(nh_);

    // calculate ur5 pose from box goal pose
    geometry_msgs::PoseStamped left_target_pose;
    left_target_pose = box_pose;
    KDL::Rotation left_rot;
    left_rot = left_rot.Quaternion(left_target_pose.pose.orientation.x, left_target_pose.pose.orientation.y, left_target_pose.pose.orientation.z, left_target_pose.pose.orientation.w);
    left_rot.DoRotY(-3.14/2);
    KDL::Vector left_shift;
    left_shift.z(-sceneManager.box_.dimensions[0]/2);
    left_shift = left_rot * left_shift;
    left_target_pose.pose.position.x = left_target_pose.pose.position.x - direction.x + left_shift.x();
    left_target_pose.pose.position.y = left_target_pose.pose.position.y - direction.y + left_shift.y();
    left_target_pose.pose.position.z = left_target_pose.pose.position.z - direction.z + left_shift.z();
    left_rot.GetQuaternion(left_target_pose.pose.orientation.x, left_target_pose.pose.orientation.y, left_target_pose.pose.orientation.z, left_target_pose.pose.orientation.w);

    // ur5 target position for placing
    ROS_INFO("moving object with both arms");
    if (!moveObject(object_id, left_target_pose, 0.1)) return false;

    // release clamp
    // switch controller
    if (!switch_controller("left_vel_based_admittance_traj_controller", "left_vel_based_pos_traj_controller", "ur5"))
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
    left_current_robot_state_ = getCurrentRobotStateMsg();
    linearMove(correct_vec, false, true, false);

    // detach object
    left_.detachObject(object_id);
    left_current_robot_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // un-grasp ur5
    graspMove(-0.05, false, true, false);

    // move ur5 to prepare for push
    try_step = true;
    while (try_step && ros::ok()) {
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
        left_waypoint.position.y = left_waypoint.position.y + sceneManager.box_.dimensions[1]/2 + 0.05;
        left_waypoint.position.z = left_waypoint.position.z;
        left_waypoints.push_back(left_waypoint);

        moveit_msgs::RobotTrajectory left_trajectory;
        double fraction = left_.computeCartesianPath(left_waypoints, 0.001, 0.0, left_trajectory, true);
        if (fraction < 0.9) {
            ROS_WARN("Left arm cartesian path. (%.2f%% achieved)", fraction * 100.0);
            try_step = try_again_question();
            if (!try_step) return false;
        }
        else {
            dual_arm_toolbox::TrajectoryProcessor::clean(left_trajectory);
            moveit::planning_interface::MoveGroup::Plan left_plan;
            left_plan.trajectory_ = left_trajectory;
            execute(left_plan);
            left_.setPoseReferenceFrame("world");
            try_step = false;
        }
    }
    try_step = true;
    while (try_step && ros::ok()) {
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
        left_rot.DoRotX(3.14/2);
        left_rot.GetQuaternion(left_waypoint.orientation.x, left_waypoint.orientation.y, left_waypoint.orientation.z, left_waypoint.orientation.w);
        left_waypoint.position.z = left_waypoint.position.z + 0.18;
        left_waypoints.push_back(left_waypoint);

        moveit_msgs::RobotTrajectory left_trajectory;
        double fraction = left_.computeCartesianPath(left_waypoints, 0.001, 0.0, left_trajectory, true);
        if (fraction < 0.9) {
            ROS_WARN("Left arm cartesian path. (%.2f%% achieved)", fraction * 100.0);
            try_step = try_again_question();
            if (!try_step) return false;
        }
        else {
            dual_arm_toolbox::TrajectoryProcessor::clean(left_trajectory);
            moveit::planning_interface::MoveGroup::Plan left_plan;
            left_plan.trajectory_ = left_trajectory;
            execute(left_plan);
            left_.setPoseReferenceFrame("world");
            try_step = false;
        }
    }

    // move closer
    graspMove(0.05, false, true, false);

    // attach object again to ur5 and update State Msg
    left_.attachObject(object_id, left_.getEndEffectorLink());
    left_current_robot_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // push box into goal position  //TODO: method linear move with vector.
    geometry_msgs::Vector3Stamped directionStamped;
    directionStamped.header.frame_id = "world";
    // transform direction vector into frame "world"
    KDL::Rotation shelf_table_rot;
    shelf_table_rot.DoRotZ(-3.14/4);
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
    left_current_robot_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // moving away from object using the same vector in opposite direction
    directionStamped.vector.x = - directionStamped.vector.x;
    directionStamped.vector.y = - directionStamped.vector.y;
    directionStamped.vector.z = - directionStamped.vector.z;
    linearMove(directionStamped, false, true, true);

    return true;
}

bool DualArmRobot::moveObject(std::string object_id, geometry_msgs::PoseStamped left_pose, double scale) {
    ROS_INFO("Moving object with both arms");
    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;
    moveit_msgs::RobotTrajectory both_arms_trajectory;
    bool adapt_error;


    // plan ur5 plan
    allowedArmCollision(true, object_id);
    try_step = true;
    moveit::planning_interface::MoveGroup::Plan left_plan;
    while (try_step && ros::ok()) {
        adapt_error = false;
        left_.setPoseTarget(left_pose, left_.getEndEffectorLink());
        left_.setStartState(left_current_robot_state_);
        error = left_.plan(left_plan);
        // get both arms trajectory
        allowedArmCollision(true,object_id);
        if (adaptTrajectory(left_plan.trajectory_, arms_offset_, both_arms_trajectory, 0.5)) ROS_INFO("successfully calculated trajectory for both arms");
        else {
            ROS_WARN("Problem adapting trajectory");
            adapt_error = true;
        }
        if (error.val != 1 || adapt_error) {
            ROS_WARN("MoveIt!Error Code: %i", error.val);
            try_step = try_again_question();
            if (!try_step) return false;
        } else {
            ROS_INFO("Moving ur5 into goal position");
            dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(left_plan.trajectory_, scale);
            try_step = false;
            allowedArmCollision(false, object_id);
        }
    }

    dual_arm_toolbox::TrajectoryProcessor::clean(both_arms_trajectory);

    // get plan from trajectory
    moveit::planning_interface::MoveGroup::Plan both_arms_plan;
    both_arms_plan.trajectory_ = both_arms_trajectory;
    both_arms_plan.start_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // visualize plan and execute
//    dual_arm_toolbox::TrajectoryProcessor::visualizePlan(both_arms_plan, 5);
    execute(both_arms_plan);

    allowedArmCollision(false,object_id);

    return true;
}

bool DualArmRobot::planMoveObject(std::string object_id, geometry_msgs::PoseStamped left_pose, double scale) {
    ROS_INFO("Moving object with both arms (planning only)");
    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;
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


    // plan ur5 plan
    allowedArmCollision(true, object_id);
    try_step = true;
    moveit::planning_interface::MoveGroup::Plan left_plan;
    before_planning_loop = ros::Time::now();
    allowedArmCollision(true,object_id);
    while (try_step && ros::ok()) {
        loops++;
        adapt_error = false;
        left_.setPoseTarget(left_pose, left_.getEndEffectorLink());
        left_.setStartState(left_current_robot_state_);

        before_planning_left = ros::Time::now();
        error = left_.plan(left_plan);
        after_planning_left = ros::Time::now();

        // get both arms trajectory
        before_planning_adaption = ros::Time::now();
        if (adaptTrajectory(left_plan.trajectory_, arms_offset_, both_arms_trajectory, 0.5)) ROS_INFO("successfully calculated trajectory for both arms");
        else {
            ROS_WARN("Problem adapting trajectory");
            adapt_error = true;
        }
        after_planning_adaption = ros::Time::now();
        if (error.val != 1 || adapt_error) {
            ROS_WARN("MoveIt!Error Code: %i", error.val);
            try_step = try_again_question();
            if (!try_step) return false;
        } else {
            dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(left_plan.trajectory_, scale);
            try_step = false;
            allowedArmCollision(false, object_id);
        }
    }
    after_planning_loop = ros::Time::now();
    dual_arm_toolbox::TrajectoryProcessor::clean(both_arms_trajectory);

    // get plan from trajectory
    moveit::planning_interface::MoveGroup::Plan both_arms_plan;
    both_arms_plan.trajectory_ = both_arms_trajectory;
    both_arms_plan.start_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    allowedArmCollision(false,object_id);
    //execute(both_arms_plan);

    // evaluation
    planning_time_left = after_planning_left - before_planning_left;
    traj_steps = both_arms_trajectory.joint_trajectory.points.size();
 //   std::cout << both_arms_trajectory.joint_trajectory.points.size();
    planning_time_adaption = after_planning_adaption - before_planning_adaption;
    planning_time_loop = after_planning_loop - before_planning_loop;

    ROS_INFO(":::::: VALUES EVALUATION ::::::");
    ROS_INFO("Planning Time for UR5 took: %li nsec", planning_time_left.toNSec());
    ROS_INFO("Planning Time for adaption took: %li nsec", planning_time_adaption.toNSec());
    ROS_INFO("Trajectory has %lu waypoints", traj_steps);
    ROS_INFO("Planning Time for loop took: %li nsec", planning_time_loop.toNSec());
    ROS_INFO("Planning loop count: %i steps", loops);

    // visualize plan
//    dual_arm_toolbox::TrajectoryProcessor::visualizePlan(both_arms_plan, 10);

    return true;
}

void DualArmRobot::allowedArmCollision(bool enable, std::string left_attachedObject) {
    // planning scene setup
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planningScene(kinematic_model);
    collision_detection::AllowedCollisionMatrix acm = planningScene.getAllowedCollisionMatrix();
    moveit_msgs::PlanningScene planningSceneMsg;

    ros::Publisher planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while(planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    // Robot links and attached object
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
    // left_links.push_back(left_attachedObject.c_str());

    std::vector<std::string> right_links;
    right_links.push_back("right_ee_link");
    right_links.push_back("right_forearm_link");
    // right_links.push_back("right_tool0");
    right_links.push_back("right_upper_arm_link");
    right_links.push_back("right_wrist_1_link");
    right_links.push_back("right_wrist_2_link");
    right_links.push_back("right_wrist_3_link");
    // right_links.push_back("right_ee_0");
    // right_links.push_back("right_ee_frame");
    // right_links.push_back("right_ee_spring");

    acm.setEntry(left_links, right_links, enable);  //Set an entry corresponding to all possible pairs between two sets of elements.

    // publish scene diff
    acm.getMessage(planningSceneMsg.allowed_collision_matrix);
    planningSceneMsg.is_diff = true;
    planning_scene_diff_publisher.publish(planningSceneMsg);
    ROS_INFO("Allowed collision between robot arms %s", enable?"ENABLED":"DISABLED");
    sleep(2);
}

bool DualArmRobot::linearMove(geometry_msgs::Vector3Stamped direction, 
                            bool avoid_collisions, 
                            bool use_left,
                            bool use_right) {

    bool try_step;
    double fraction;

    // left arm
    ROS_INFO("Let left arm move straight line");
    if (use_left) try_step = true;
    while (try_step && ros::ok()) {
        left_.setPoseReferenceFrame(direction.header.frame_id);
        left_.setStartState(left_current_robot_state_);
        // fk service client setup
        ros::ServiceClient fk_client = nh_.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");
        // A service definition for a standard forward kinematics service
        // The frame_id in the header messages is the frame in which the forward kinematics poses will be returned.
        moveit_msgs::GetPositionFK fk_msg;
        fk_msg.request.header.frame_id = direction.header.frame_id;
        fk_msg.request.fk_link_names.push_back(left_.getEndEffectorLink());
        fk_msg.request.robot_state = left_current_robot_state_;
        fk_client.call(fk_msg.request, fk_msg.response);
        // get virtual pose from virtual state
        if (fk_msg.response.error_code.val != 1) {
            ROS_WARN("fk request error");
            return false;
        }
        geometry_msgs::PoseStamped left_start_pose = fk_msg.response.pose_stamped[0];

        // waypoints
        std::vector<geometry_msgs::Pose> left_waypoints;
        geometry_msgs::Pose left_waypoint = left_start_pose.pose;
        left_waypoints.push_back(left_waypoint);
        left_waypoint.position.x = left_waypoint.position.x + direction.vector.x;
        left_waypoint.position.y = left_waypoint.position.y + direction.vector.y;
        left_waypoint.position.z = left_waypoint.position.z + direction.vector.z;

        left_waypoints.push_back(left_waypoint);
        moveit_msgs::RobotTrajectory left_trajectory;
        fraction = left_.computeCartesianPath(left_waypoints, 0.001, 0.0, left_trajectory, avoid_collisions);
        if (fraction < 0.9) {
            ROS_WARN("Left arm cartesian path. (%.2f%% achieved)", fraction * 100.0);
            try_step = try_again_question();
            if (!try_step) return false;
        }
        else {
            dual_arm_toolbox::TrajectoryProcessor::clean(left_trajectory);
            moveit::planning_interface::MoveGroup::Plan left_plan;
            std::cout << "joint_names: " 
                << left_trajectory.joint_trajectory.joint_names[0] << "  "
                << left_trajectory.joint_trajectory.joint_names[1] << "  "
                << left_trajectory.joint_trajectory.joint_names[2] << "  "
                << left_trajectory.joint_trajectory.joint_names[3] << "  "
                << left_trajectory.joint_trajectory.joint_names[4] << "  "
                << left_trajectory.joint_trajectory.joint_names[5] << std::endl;
            left_plan.trajectory_ = left_trajectory;
            bool success = execute(left_plan);
            if(!success)
                ROS_WARN("Left arm trajectory failed! ");
            try_step = false;
        }
    }
    
    ROS_INFO("Let right arm move straight line");
    // right arm
    if (use_right) try_step = true;
    while (try_step && ros::ok()) {
        right_.setStartStateToCurrentState();
        std::vector<geometry_msgs::Pose> right_waypoints;
        right_.setPoseReferenceFrame(direction.header.frame_id);
        geometry_msgs::Pose right_waypoint = right_.getCurrentPose(right_.getEndEffectorLink()).pose;
        right_waypoints.push_back(right_waypoint);

        // calculate waypoint
        right_waypoint.position.x = right_waypoint.position.x + direction.vector.x;
        right_waypoint.position.y = right_waypoint.position.y + direction.vector.y;
        right_waypoint.position.z = right_waypoint.position.z + direction.vector.z;

        right_waypoints.push_back(right_waypoint);
        moveit_msgs::RobotTrajectory right_trajectory;
        fraction = right_.computeCartesianPath(right_waypoints, 0.001, 0.0, right_trajectory, avoid_collisions);
        if (fraction < 0.9) {
            ROS_WARN("Right arm cartesian path. (%.2f%% achieved)", fraction * 100.0);
            try_step = try_again_question();
            if (!try_step) return false;
        } else {
            moveit::planning_interface::MoveGroup::Plan right_plan;
            dual_arm_toolbox::TrajectoryProcessor::clean(right_trajectory);
            right_plan.trajectory_ = right_trajectory;
            bool success = execute(right_plan);
            if(!success)
                ROS_WARN("Right arm trajectory failed! ");
            try_step = false;
        }
    }
    
}

bool DualArmRobot::execute(moveit::planning_interface::MoveGroup::Plan plan) {
    ROS_INFO("executing trajectory");
#ifndef OFFLINE
    moveit::planning_interface::MoveGroup::Plan plan_left;
    moveit::planning_interface::MoveGroup::Plan plan_right;

    dual_arm_toolbox::TrajectoryProcessor::split(plan.trajectory_, plan_left.trajectory_,plan_right.trajectory_,"left","right");
    dual_arm_toolbox::TrajectoryProcessor::clean(plan_left.trajectory_);
    dual_arm_toolbox::TrajectoryProcessor::clean(plan_right.trajectory_);

    bool success_right;
    bool success_left;

    moveit_simple_controller_manager::FollowJointTrajectoryControllerHandle handle_left(left_controller_,"follow_joint_trajectory");
    moveit_simple_controller_manager::FollowJointTrajectoryControllerHandle handle_right(right_controller_,"follow_joint_trajectory");
    //moveit_simple_controller_manager::FollowJointTrajectoryControllerHandle handle_right("right/right_vel_based_traj_admittance_controller","follow_joint_trajectory");

    // for both arms trajectories: because in planning each arm was not aware of the other there is a collision check before executing the trajectory
    /* TODO: put in again later
    if ((plan_left.trajectory_.joint_trajectory.joint_names.size() > 0) && (plan_right.trajectory_.joint_trajectory.joint_names.size() > 0)){
        // check trajectory for collisions
        robot_model_loader::RobotModelLoader robot_model_loader(
                "robot_description");
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        planning_scene::PlanningScene planningScene(kinematic_model);
        bool isValid = planningScene.isPathValid(plan.start_state_,plan.trajectory_,"arms");
        if (!isValid){
            ROS_ERROR("Path is invalid. Execution aborted");
            return false;
        }
        else ROS_INFO("Checked path. Path is valid. Executing...");
    }*/
    /// Print out the joint trajectory infomation
    if (plan_left.trajectory_.joint_trajectory.joint_names.size() > 0){
        ROS_INFO("Trajectory sent to left arm");
        success_left = handle_left.sendTrajectory(plan_left.trajectory_);
    }

    if (plan_right.trajectory_.joint_trajectory.joint_names.size() > 0){
        ROS_INFO("Trajectory sent to right arm");
        success_right = handle_right.sendTrajectory(plan_right.trajectory_);
    }

    if (plan_left.trajectory_.joint_trajectory.joint_names.size() > 0) 
        success_left = handle_left.waitForExecution();
    else success_left = true;
    if (plan_right.trajectory_.joint_trajectory.joint_names.size() > 0) 
        success_right = handle_right.waitForExecution();
    else success_right = true;
    sleep(0.5);  // to be sure robot is at goal position

    // update last goal ur5
    if (plan_left.trajectory_.joint_trajectory.joint_names.size()) {
        left_current_robot_state_.is_diff = true;
        left_current_robot_state_.joint_state.effort = plan_left.trajectory_.joint_trajectory.points[
                plan_left.trajectory_.joint_trajectory.points.size() - 1].effort;
        left_current_robot_state_.joint_state.header = plan_left.trajectory_.joint_trajectory.header;
        left_current_robot_state_.joint_state.name = plan_left.trajectory_.joint_trajectory.joint_names;
        left_current_robot_state_.joint_state.position = plan_left.trajectory_.joint_trajectory.points[
                plan_left.trajectory_.joint_trajectory.points.size() - 1].positions;
        left_current_robot_state_.joint_state.velocity = plan_left.trajectory_.joint_trajectory.points[
                plan_left.trajectory_.joint_trajectory.points.size() - 1].velocities;
    }

    // fk service client setup
    ros::ServiceClient fk_client = nh_.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");
    moveit_msgs::GetPositionFK fk_msg;
    fk_msg.request.header.frame_id = "world";
    fk_msg.request.fk_link_names.push_back(left_.getEndEffectorLink());
    fk_msg.request.robot_state = left_current_robot_state_;
    fk_client.call(fk_msg.request, fk_msg.response);

    // get virtual pose from virtual state
    if (fk_msg.response.error_code.val != 1) {
        ROS_WARN("fk request error");
        return false;
    }
    left_current_pose_ = fk_msg.response.pose_stamped[0];

    return success_left&&success_right;
#endif
#ifdef OFFLINE

    double error = arms_.execute(plan);
    sleep(2); // to be sure robot is at goal position
    left_current_pose_ = left_.getCurrentPose(left_.getEndEffectorLink());
    left_current_robot_state_.is_diff = true;
    left_current_robot_state_.joint_state.position = left_.getCurrentJointValues();
    left_current_robot_state_.joint_state.name = left_.getJointNames();
    if (error == -1) return false;
    return true;
#endif
}

// Get the joint angles
// groupName: "left_manipulator" "right_manipulator" "arms"
std::vector<double> DualArmRobot::getJointAngles(std::string groupName){
    ROS_INFO("-------- Get Joint Angles-------------------------");
    // setup planning scene
    // Look up the robot description on the ROS parameter server and construct a RobotModel to use
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame %s", kinematic_model->getModelFrame().c_str());
    planning_scene::PlanningScene planningScene(kinematic_model);

    // setup JointModelGroup
    //ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    // Construct a RobotState that maintains the configuration of the robot.
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(groupName);
    const std::vector<std::string>& joint_names = joint_model_group->getJointModelNames();
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i=0; i<joint_names.size(); ++i){
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    return joint_values;


}
bool DualArmRobot::moveHome() {
    ROS_INFO("Moving arms into home position - both arms up");
    sleep(5);
    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;

    // move left arm
    // try_step = true;
    // while (try_step && ros::ok()) {
    //     // Set the current joint values to be ones previously remembered by rememberJointValues() or, 
    //     // if not found, that are specified in the SRDF under the name "left_up" as a group state. 
    //     left_.setNamedTarget("left_up");
    //     geometry_msgs::PoseStamped targetPose = left_.getPoseTarget(); 	
    //     moveit::planning_interface::MoveGroup::Plan left_plan;
    //     left_.setStartState(left_current_robot_state_);
    //     error = left_.plan(left_plan);
    //     if (error.val != 1) {
    //         ROS_WARN("MoveIt!Error Code: %i", error.val);
    //         try_step = try_again_question();
    //         if (!try_step) return false;
    //     } else {
    //         ROS_INFO("Moving left arm into home position");
            
    //         dual_arm_toolbox::TrajectoryProcessor::clean(left_plan.trajectory_);
    //         dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(left_plan.trajectory_, 0.4);
 
    //         // left_.setJointValueTarget("left_shoulder_pan_joint", 1.57);
    //         // left_.setJointValueTarget("left_shoulder_lift_joint", -2.356);
    //         // left_.setJointValueTarget("left_elbow_joint", 0);
    //         // left_.setJointValueTarget("left_wrist_1_joint", -1.57);
    //         // left_.setJointValueTarget("left_wrist_2_joint", 1.57);
    //         // left_.setJointValueTarget("left_wrist_3_joint", 0);

    //         // //left__.setJointValueTarget(ur10JointTarget);
    //         // // left_.plan(plan);
    //         // ROS_WARN("visualizing plan. STRG+C to interrupt.");
    //         sleep(4);
    //         execute(left_plan);
    //         sleep(3);
    //         try_step = false;

    //     }
    // }

    // // move right arm
    // try_step = true;
    // while (try_step && ros::ok()) {
    //     right_.setNamedTarget("right_up");
    //     moveit::planning_interface::MoveGroup::Plan right_plan;
    //     error = right_.plan(right_plan);
    //     if (error.val != 1) {
    //         ROS_WARN("MoveIt!Error Code: %i", error.val);
    //         try_step = try_again_question();
    //         if (!try_step) return false;
    //     } else {
    //         ROS_INFO("Moving right arm into home position");
    //         dual_arm_toolbox::TrajectoryProcessor::clean(right_plan.trajectory_);
    //         dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(right_plan.trajectory_, 0.4);

    //         // short distance postion
    //         // right_.setJointValueTarget("right_shoulder_pan_joint", 1.57);
    //         // right_.setJointValueTarget("right_shoulder_lift_joint", -2.356);
    //         // right_.setJointValueTarget("right_elbow_joint", 0);
    //         // right_.setJointValueTarget("right_wrist_1_joint", -1.57);
    //         // right_.setJointValueTarget("right_wrist_2_joint", 1.57);
    //         // right_.setJointValueTarget("right_wrist_3_joint", 0);

    //         // //right_.setJointValueTarget(ur10JointTarget);
    //         // //right_.plan(plan);
    //         // ROS_WARN("visualizing plan. STRG+C to interrupt.");
    //         sleep(4);
    //         execute(right_plan);
    //         // right_.execute(right_plan);
    //         sleep(3);
    //         try_step = false;
    //     }
    // }

    // move both at simultaneously

    // moveit_msgs::RobotState robot_grasp_state_;
    // robot_grasp_state_ = getCurrentRobotStateMsg();
    // geometry_msgs::PoseStamped  left_grasp_pose_ = left_.getCurrentPose(left_.getEndEffectorLink());
    // geometry_msgs::PoseStamped  right_grasp_pose_ = right_.getCurrentPose(right_.getEndEffectorLink());
    try_step = true;
    while (try_step && ros::ok()) {
        arms_.setNamedTarget("arms_up");
        moveit::planning_interface::MoveGroup::Plan arms_plan;
        arms_.setStartState(left_current_robot_state_);
        error = arms_.plan(arms_plan);
        if (error.val != 1) {
            ROS_WARN("MoveIt!Error Code: %i", error.val);
            try_step = try_again_question();
            if (!try_step) return false;
        } else {
            ROS_INFO("Moving arms into home position");
            dual_arm_toolbox::TrajectoryProcessor::clean(arms_plan.trajectory_);
            dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(arms_plan.trajectory_, 0.4);
            execute(arms_plan);
            try_step = false;
            ROS_INFO("Already in the home position");
        }
        
        // sleep(2);
        // moveit_msgs::RobotState left_current_robot_state_temp_ = getCurrentRobotStateMsg();
        // arms_.setStartState(left_current_robot_state_temp_);
        
        // arms_.setPoseTarget(left_grasp_pose_, left_.getEndEffectorLink());
        // arms_.setPoseTarget(right_grasp_pose_, right_.getEndEffectorLink());
        // // The representation of a motion plan (as ROS messages), it's a structure.
        // // THe method of plan(), compute a motion plan that takes the group declared in the constructor 
        // // from the current state to the specified target.
        // moveit::planning_interface::MoveGroup::Plan arms_plan_temp;
        // error = arms_.plan(arms_plan_temp);
        // if (error.val != 1) {
        //     ROS_WARN("MoveIt!Error Code: %i", error.val);
        //     try_step = try_again_question();
        //     if (!try_step) return false;
        // } else {
        //     ROS_INFO("Moving arms into start position");
        //     dual_arm_toolbox::TrajectoryProcessor::clean(arms_plan_temp.trajectory_);
        //     dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(arms_plan_temp.trajectory_, 0.4);
        //     execute(arms_plan_temp);
        //     try_step = false;
        // }
    }
    return true;
}
