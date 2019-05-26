//
// Created by Chunting  on 07.10.18.
//

// ROS
#include <ros/ros.h>

// Standard
#include <stdio.h>
#include <iostream>

// KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp> //Newton-Raphson //takes joint limits into account
#include <kdl/chainiksolverpos_nr.hpp>    //Newton-Raphson

// Message
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

// PID
#include <control_toolbox/pid.h>
// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

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

using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
class AdmittanceControl
{
  protected:
    // ROS VARIABLES:
    // A handle to the node in ros
    ros::NodeHandle nh_;
    // Rate of the run loop
    // ros::Rate loop_rate_;

    // KDL
    KDL::Chain kdl_chain_;

    // Subscribers:
    // wrench
    // Subscriber for the arm state
    ros::Subscriber sub_offset_new_;
    // Subscriber for the ft sensor at the endeffector
    ros::Subscriber sub_wrench_external_;
    // Subscriber for the offset of the attractor
    ros::Subscriber sub_offset_desired_; // Should be a Service



    robot_model_loader::RobotModelLoader robotModelLoader;
    robot_model::RobotModelPtr kinematic_modelPtr;
    planning_scene::PlanningScenePtr planningScenePtr;
    robot_state::RobotStatePtr kinematic_statePtr;
    const robot_state::JointModelGroup *joint_model_groupPtr;
    // moveit::planning_interface::MoveGroupInterface arm_group;

   


    

    void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    void offsetCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void desiredOffsetCallback(const geometry_msgs::PointStamped::ConstPtr &msg);

    // runtime variables
    double delta_z_, delta_x_, delta_y_;          
    KDL::JntArray q_last_; // last joint state
    std::vector<double> new_start_position_;
    KDL::Vector force_;

    // Parameters
    double max_vel_;
    double max_shift_;
    double contact_F;
    // FORCE/TORQUE-SENSOR FILTER:
    // Parameters for the noisy wrench
    double wrench_filter_factor_;
    double force_dead_zone_thres_;
    double torque_dead_zone_thres_;
    // external wrench (force/torque sensor) in "robotiq_ft_frame_id" frame
    Vector6d wrench_external_;
    // receiving a new equilibrium from a topic
    Vector3d offset_new_;
    Vector3d offset_desired_;

    // Control, PID
    double wrench_tolerance_;
    control_toolbox::Pid pid_controller_;
    control_toolbox::Pid::Gains gains_;

    std::string wrench_topic;
    std::string offset_topic;
    std::string root_name;
    std::string tip_name;
    std::string joint_model_group;

  public:
    AdmittanceControl();
    ~AdmittanceControl();
    void init(ros::NodeHandle &nh);
    void starting();
    void update_admittance_state(std::vector<double> &position, const ros::Duration &period);
    void set_shift(double d);

  private:
};

AdmittanceControl::AdmittanceControl()
{
}

AdmittanceControl::~AdmittanceControl()
{
}
// Read from force torqued sensor
void AdmittanceControl::wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{ // wrench in FT sensor frame
    geometry_msgs::WrenchStamped wrench_msg = *msg;
    Vector6d wrench_ft_frame;
    // Reading the FT-sensor in its own frame (robotiq_ft_frame_id)
    wrench_ft_frame << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

    for (int i = 0; i < 3; i++)
    {
        if (abs(wrench_ft_frame(i)) < force_dead_zone_thres_)
        {
            wrench_ft_frame(i) = 0;
        }
        if (abs(wrench_ft_frame(i + 3)) < torque_dead_zone_thres_)
        {
            wrench_ft_frame(i + 3) = 0;
        }
    }
    // Filter and update
    wrench_external_ << (1 - wrench_filter_factor_) * wrench_external_ +
                            wrench_filter_factor_ * wrench_ft_frame;
    force_.x(wrench_external_(0));
    force_.y(wrench_external_(1));
    force_.z(wrench_external_(2));
}

void AdmittanceControl::offsetCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    offset_new_ << msg->point.x, msg->point.y, msg->point.z;
}

void AdmittanceControl::desiredOffsetCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    offset_desired_ << msg->point.x, msg->point.y, msg->point.z;
    ROS_INFO_STREAM("I heared the desired offset " << offset_desired_);
}
// std::string getprefix(const ros::NodeHandle &nh)
// {
// 	const std::string complete_ns = nh.getNamespace();
// 	std::size_t first = complete_ns.find_first_of("/");
//     std::size_t last = complete_ns.find_last_of("/");
// 	std::string prefix = complete_ns.substr(first+1, last-first-1);
//     return prefix+"_";
// }

void AdmittanceControl::init(ros::NodeHandle &nh)
{

    nh_ = nh;
    // init variables
    delta_z_ = 0.0;
    delta_x_ = 0.0;
    delta_y_ = 0.0;
    new_start_position_.clear();

    // KDL
    KDL::Tree kdl_tree;
    std::string robot_desc_string;
   
    std::string name_space = nh.getNamespace(); // /right/vel_based_admittance_traj_controller
    // std::string prefix = getprefix(nh);  // Get prefix from name_space, i.e., left_ or right_
    std::cout << "--------------------> name_space:  " << name_space << std::endl;

    // initializing the class variables
    wrench_external_.setZero();
    offset_new_.setZero();
    offset_desired_.setZero();
    nh.param("joint_model_group", joint_model_group, std::string("right_manipulator"));
    nh.param("root_name", root_name, std::string());
    nh.param("tip_name", tip_name, std::string());
    nh.param("admittance_pid/p", gains_.p_gain_, 0.01);
    nh.param("admittance_pid/d", gains_.d_gain_, 0.01);
    nh.param("admittance_pid/i_clamp_max", gains_.i_max_, 0.0);
    nh.param("admittance_pid/i_clamp_min", gains_.i_min_, 0.0);

    // root_name = prefix + root_name;
    // tip_name = prefix + tip_name;
    std::cout << "root name " << root_name << "\ttip name  " << tip_name << std::endl;
    std::cout << "Namespace: " << name_space << std::endl;
    std::cout << "p = " << gains_.p_gain_ << "  d =   " << gains_.d_gain_ << std::endl;


    robotModelLoader = robot_model_loader::RobotModelLoader("robot_description");
    kinematic_modelPtr = robotModelLoader.getModel();
    ROS_INFO("MoveIt Model Frame: %s \n", kinematic_modelPtr->getModelFrame().c_str());
   
    kinematic_statePtr = std::make_shared<moveit::core::RobotState>(kinematic_modelPtr);
    // planning_scene::PlanningScenePtr planningScenePtr = std::make_shared<planning_scene::PlanningScene>(kinematic_modelPtr);
   
    joint_model_groupPtr = kinematic_modelPtr->getJointModelGroup(joint_model_group);
    // arm_group = moveit::planning_interface::MoveGroupInterface(joint_model_group);
    // arm_group.res
    const std::vector<std::string> &joint_names = joint_model_groupPtr->getJointModelNames();
    std::vector<double> joint_values;
    kinematic_statePtr->copyJointGroupPositions(joint_model_groupPtr, joint_values);
    for(std::size_t i=0; i<joint_names.size(); ++i){
        ROS_INFO("%s : %f", joint_names[i].c_str(), joint_values[i]);
    }
    kinematic_statePtr->setJointGroupPositions(joint_model_groupPtr, joint_values);
    const Eigen::Affine3d &end_effector_state = kinematic_statePtr->getGlobalLinkTransform(tip_name);
    ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
    bool found_ik = kinematic_statePtr->setFromIK(joint_model_groupPtr, end_effector_state, 10, 0.1);
    joint_values.clear();
    if(found_ik){
        kinematic_statePtr->copyJointGroupPositions(joint_model_groupPtr, joint_values);
        for(std::size_t i=0; i<joint_names.size(); ++i){
            ROS_INFO("%s : %f", joint_names[i].c_str(), joint_values[i]);
        }
    }else{
        ROS_WARN("Did not find IK solution");
    }
    
    

    nh.param("/robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree))
    {
        ROS_ERROR("Failed to construct kdl tree in namespace: %s", name_space.c_str());
    }
    else
        ROS_INFO("Got kdl tree for joint admittance controller");



    if (!kdl_tree.getChain(root_name, tip_name, kdl_chain_))
        ROS_ERROR("Failed to construct kdl chain with root: %s and tip: %s in namespace %s", root_name.c_str(), tip_name.c_str(), name_space.c_str());
    else
        ROS_INFO("Got kdl chain for joint admittance controller");

    // Settings
    nh.param("max_velocity", max_vel_, double());
    nh.param("max_shift", max_shift_, double());
    nh.param("contact_force", contact_F, double());
    // Set up dead zone and filter factor
    nh.param("force_dead_zone_thres", force_dead_zone_thres_, double());
    nh.param("torque_dead_zone_thres", torque_dead_zone_thres_, double());
    nh.param("wrench_filter_factor", wrench_filter_factor_, double());
    contact_F = -std::abs(contact_F); // must be a negative value

    // PID
    if (!pid_controller_.init(ros::NodeHandle(nh_, "admittance_pid")))
    {
        ROS_ERROR("Admittance Control: could not construct PID controller");
    }
    pid_controller_.reset();
    pid_controller_.setGains(gains_);

    // wrench

    nh.param("wrench_tolerance", wrench_tolerance_, double());
    nh.param("wrench_topic", wrench_topic, std::string());
    // nh.param("offset_topic", offset_topic, std::string());
    sub_wrench_external_ = nh_.subscribe(wrench_topic, 1, &AdmittanceControl::wrenchCallback, this);

    // sub_offset_new_ = nh_.subscribe("/offset_pose_state", 10, &AdmittanceControl::offsetCallback, this);
    // sub_offset_desired_ = nh_.subscribe("/desired_offset_point", 1, &AdmittanceControl::desiredOffsetCallback, this);
}

void AdmittanceControl::starting()
{
    ROS_INFO("Starting Admittance Controller");
    set_shift(0);
    q_last_.resize(0);
}

void AdmittanceControl::update_admittance_state(std::vector<double> &position, const ros::Duration &period)
{
    // Solver
    // KDL::ChainIkSolverVel_pinv
    // An inverse velocity kinematics algorithm based on the generalize pseudo inverse to calculate the
    // velocity transformation from Cartesian to joint space of a general KDL::Chain.
    // It uses a svd-calculation based on householders rotations.
    KDL::ChainIkSolverVel_pinv ikSolverVel(kdl_chain_);
    KDL::ChainFkSolverPos_recursive fkSolverPos(kdl_chain_);
    KDL::ChainIkSolverPos_NR ikSolverPosNR(kdl_chain_, fkSolverPos, ikSolverVel, 100, 1e-6);

    // Joint Array in
    unsigned int nj = kdl_chain_.getNrOfJoints();
    // for (std::size_t i = 0; i < kdl_chain_.getNrOfSegments(); i++)
	// {
	//     ROS_INFO_STREAM("segment(" << i << "): " << kdl_chain_.getSegment(i).getName());
	// }

    KDL::JntArray q_in(nj);
    std::vector<double> joint_values;
    ROS_INFO("Input joint positions: " );
    for (int i = 0; i < nj; i++)
    {
        q_in(i, 0) = position[i];
        joint_values.push_back(position[i]);
        ROS_INFO("%f", q_in(i, 0)*180/3.14);
    }
    kinematic_statePtr->setJointGroupPositions(joint_model_groupPtr, joint_values);
    const Eigen::Affine3d &end_effector_state = kinematic_statePtr->getGlobalLinkTransform(tip_name);
    ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
    bool found_ik = kinematic_statePtr->setFromIK(joint_model_groupPtr, end_effector_state, 10, 0.1);
    joint_values.clear();
    if(found_ik){
        kinematic_statePtr->copyJointGroupPositions(joint_model_groupPtr, joint_values);
        for(std::size_t i=0; i<joint_values.size(); ++i){
            ROS_INFO("%f", joint_values[i]);
        }
    }else{
        ROS_WARN("Did not find IK solution");
    }

    // forward kinematics
    KDL::Frame p_eef;
    Eigen::Affine3d eePose_eig;
    int fk_feedback = fkSolverPos.JntToCart(q_in, p_eef); // segmentNr = -1
    double yaw = 0;    // Z-axis
    double pitch = 0;  // Y-axis
    double roll = 0;   // X-axis
    
    ROS_INFO_STREAM("kdl p_eef " <<  p_eef.p << " " << yaw << " " << pitch << " " << roll );
    if (fk_feedback < 0)
    {
        ROS_WARN("Admittance Control: Problem solving forward kinematics. Error: %s",
                 fkSolverPos.strError(fk_feedback));
        return;
    }

    // wrench at eef frame
    KDL::Vector wrench_eef; // wrench in destination Frame eef
    ros::spinOnce();        // receive wrench msg
    // wrench_eef = p_eef.M.Inverse() * force_; // Rotate to get Force in coordinate system of the eef
    wrench_eef = force_; // The oritation of robotiq_frame_id frame is consident with left/right_tool0 

    // PID
    ROS_INFO("(wrench_eef.z() = %f, contact_F = %f, wrench_tolerance_ = %f", wrench_eef.z(), contact_F, wrench_tolerance_);
    if (std::abs(wrench_eef.z() - contact_F) > wrench_tolerance_)
    {
        // if (((wrench_eef.z() < contact_F) && (delta_z_ > -max_shift_)) ||
        //     ((wrench_eef.z() > contact_F) && (delta_z_ < max_shift_)))
        // {
            double pid_vel = pid_controller_.computeCommand(wrench_eef.z() - contact_F, period);
            
            if (std::abs(pid_vel) < max_vel_)
                delta_z_ = delta_z_ + pid_vel * period.toSec();
            else
                delta_z_ = delta_z_ + (std::abs(pid_vel) / pid_vel) * max_vel_ * period.toSec();
            ROS_INFO("pid_vel = %f delta_z_ = %f", pid_vel, delta_z_);
        //}
    }

    if (std::abs(wrench_eef.x()) > 10)
    {
        // if (((wrench_eef.z() < contact_F) && (delta_z_ > -max_shift_)) ||
        //     ((wrench_eef.z() > contact_F) && (delta_z_ < max_shift_)))
        // {
            double pid_vel = pid_controller_.computeCommand(wrench_eef.x(), period);
            
            if (std::abs(pid_vel) < max_vel_)
                delta_x_ = delta_x_ + pid_vel * period.toSec();
            else
                delta_x_ = delta_x_ + (std::abs(pid_vel) / pid_vel) * max_vel_ * period.toSec();
            ROS_INFO("pid_vel = %f delta_x_ = %f", pid_vel, delta_x_);
        //}
    }

    // distance is transformed to be along z-axis of eef
    KDL::Vector vec_d; // distance vector, in ee frame
    vec_d.x(delta_x_);
    vec_d.y(0);
    vec_d.z(delta_z_);
    vec_d = p_eef.M * vec_d; // Rotate distance vector, in base frame

    // target frame
    KDL::Frame p_target = p_eef;
    p_target.p = p_eef.p + vec_d; // Add distance Vector to eef Frame

    // target joint state
    KDL::JntArray q_dest(nj);

    // inverse kinematics
    KDL::JntArray q_init;
    if (q_last_.rows() == nj)
        q_init = q_last_;
    else
        q_init = q_in; // use input state only first time. The last state should be closer to the solution in other cases
    int ik_feedback = ikSolverPosNR.CartToJnt(q_init, p_target, q_dest);
    if (ik_feedback < 0)
    {
        ROS_WARN("Admittance Control: Problem solving inverse kinematics. Error: %s", ikSolverPosNR.strError(ik_feedback));
        q_dest = q_init;
    }
    ROS_INFO_STREAM("Target Frame " << p_target);

    // check for jerk motion
    for (unsigned int i = 0; i < nj; i++)
    {
        if (std::abs(q_dest(i, 0) - q_init(i, 0)) > 0.3)
        {
            ROS_WARN("Admittance Control: Jerk motion detected! Stop immediately!");
            q_dest = q_in;
            return;
        }
    }
    q_last_ = q_dest;

    // write position
    for (unsigned int i = 0; i < nj; i++)
    {
        position[i] = q_dest(i, 0);
    }
    // only position information necessary, because only the position will be forwarded to hardware_interface::setCommand()
}

void AdmittanceControl::set_shift(double d)
{
    delta_z_ = d;
    ROS_WARN("Set shift to %f", d);
}