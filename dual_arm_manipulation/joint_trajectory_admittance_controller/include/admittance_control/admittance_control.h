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

    void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    void offsetCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void desiredOffsetCallback(const geometry_msgs::PointStamped::ConstPtr &msg);

    // runtime variables
    double d_;             // distance
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

    std::string wrench_topic;
    std::string offset_topic;

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
    d_ = 0.0;
    new_start_position_.clear();

    // KDL
    KDL::Tree kdl_tree;
    std::string robot_desc_string;
    std::string root_name;
    std::string tip_name;
    std::string name_space = nh.getNamespace(); // /right/vel_based_admittance_traj_controller
    // std::string prefix = getprefix(nh);  // Get prefix from name_space, i.e., left_ or right_
    std::cout << "--------------------> name_space:  " << name_space << std::endl;

    // initializing the class variables
    wrench_external_.setZero();
    offset_new_.setZero();
    offset_desired_.setZero();

    nh.param("/robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree))
    {
        ROS_ERROR("Failed to construct kdl tree in namespace: %s", name_space.c_str());
    }
    else
        ROS_INFO("Got kdl tree for joint admittance controller");

    nh.param("root_name", root_name, std::string());
    nh.param("tip_name", tip_name, std::string());
    // root_name = prefix + root_name;
    // tip_name = prefix + tip_name;
    std::cout << "root name " << root_name << "\ttip name  " << tip_name << std::endl;
    std::cout << "Namespace: " << name_space << std::endl;

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

    // wrench

    nh.param("wrench_tolerance", wrench_tolerance_, double());
    nh.param("wrench_topic", wrench_topic, std::string());
    // nh.param("offset_topic", offset_topic, std::string());
    sub_wrench_external_ = nh_.subscribe(wrench_topic, 1, &AdmittanceControl::wrenchCallback, this);

    sub_offset_new_ = nh_.subscribe("/real_time_offset_point", 10, &AdmittanceControl::offsetCallback, this);
    sub_offset_desired_ = nh_.subscribe("/desired_offset_point", 1, &AdmittanceControl::desiredOffsetCallback, this);
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

    KDL::JntArray q_in(nj);

    for (int i = 0; i < nj; i++)
    {
        q_in(i, 0) = position[i];
    }

    // forward kinematics
    KDL::Frame p_eef;
    int fk_feedback = fkSolverPos.JntToCart(q_in, p_eef); // segmentNr = -1
    ROS_INFO_STREAM("kdl p_eef " << p_eef);
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
    wrench_eef = force_; // Force in force torque coordinate system
    // distance is described along X-axis in right ee coordinated system.
    // If d_ >0, robot looses the object, grip more tightly otherwise.
    d_ = offset_new_(0) - offset_desired_(0);

    // PID
    if (std::abs(wrench_eef.z() - contact_F) > wrench_tolerance_)
    {
        if (((wrench_eef.z() < contact_F) && (d_ > -max_shift_)) ||
            ((wrench_eef.z() > contact_F) && (d_ < max_shift_)))
        {
            double pid_vel = pid_controller_.computeCommand(wrench_eef.z() - contact_F, period);
            if (std::abs(pid_vel) < max_vel_)
                d_ = d_ + pid_vel * period.toSec();
            else
                d_ = d_ + (std::abs(pid_vel) / pid_vel) * max_vel_ * period.toSec();
        }
    }

    // distance is transformed to be along z-axis of eef
    KDL::Vector vec_d; // distance vector
    vec_d.x(0);
    vec_d.y(0);
    vec_d.z(d_);
    vec_d = p_eef.M * vec_d; // Rotate distance vector

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
    d_ = d;
    ROS_WARN("Set shift to %f", d);
}