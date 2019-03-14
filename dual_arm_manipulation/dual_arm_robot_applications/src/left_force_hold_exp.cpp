#include <moveit/move_group_interface/move_group.h>
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
// force_torque_sensor
#include "robotiq_force_torque_sensor/ft_sensor.h"
#include "robotiq_force_torque_sensor/sensor_accessor.h"

#include <dual_arm_demonstrator_iml/SceneManager.h>
#include "dual_arm_demonstrator_iml/DualArmRobot.h"

class Subscriber{  //handles callbacks and saves last received messages
protected:
    ros::Subscriber wrench_sub_;
    ros::NodeHandle nh_;
public:
    Subscriber(ros::NodeHandle& nh);
    robotiq_force_torque_sensor::ft_sensor last_wrench_msg_;
private:
    void wrenchCallback(const robotiq_force_torque_sensor::ft_sensor::Ptr& msg);
};

Subscriber::Subscriber(ros::NodeHandle& nh) : nh_(nh){

    wrench_sub_ = nh_.subscribe("/robotiq_force_torque_sensor", 1, &Subscriber::wrenchCallback, this);
    ros::ServiceClient client = nh_.serviceClient<robotiq_force_torque_sensor::sensor_accessor>("robotiq_force_torque_sensor_acc");
    robotiq_force_torque_sensor::sensor_accessor srv;
   
    if(ros::ok()){
         srv.request.command = "SET ZRO";
	    if(client.call(srv)){
		    ROS_INFO("ret: %s", srv.response.res.c_str());
	    }
    }
}

void Subscriber::wrenchCallback(const robotiq_force_torque_sensor::ft_sensor::Ptr& msg){
    last_wrench_msg_ = *msg;
    // ROS_INFO("I heard: FX[%f] FY[%f] FZ[%f]", last_wrench_msg_.Fx, last_wrench_msg_.Fy, last_wrench_msg_.Fz);

}


int main(int argc, char **argv)
{
    // ROS Setup
    ros::init(argc, argv, "ur_const_vel_publisher");
    ros::NodeHandle nh;
    ros::Publisher left_speed_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/left/ur_driver/joint_speed", 1);
    ros::AsyncSpinner asyncSpinner(2);
    asyncSpinner.start();

    Subscriber subscriber(nh);
    // Controller Interface
    std::string left_controller_ = "left/left_vel_based_pos_traj_controller";
    // MoveGroup
    moveit::planning_interface::MoveGroup left_("left_manipulator");
    moveit::planning_interface::MoveGroup::Plan plan;
    left_.setPlanningTime(30);

    ROS_WARN("robot is moving without collision checking. BE CAREFUL!");
    ROS_INFO("waiting 10 Seconds. Press Ctrl-C if Robot is in the wrong start position");
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
    ros::Duration(10).sleep();

    //long distance position
    /*
    left_.setJointValueTarget("elbow_joint", -1.366541959239651);
    left_.setJointValueTarget("shoulder_lift_joint", -2.573810648739345);
    left_.setJointValueTarget("shoulder_pan_joint", 0.5943102022167164);
    left_.setJointValueTarget("wrist_1_joint", -0.7533539281232803);
    left_.setJointValueTarget("wrist_2_joint", -0.0);
    left_.setJointValueTarget("wrist_3_joint", 0.00015758105264953662);*/

    // short distance postion
    left_.setJointValueTarget("left_shoulder_pan_joint", 1.6);
    left_.setJointValueTarget("left_shoulder_lift_joint", -1.876);
    left_.setJointValueTarget("left_elbow_joint", 1.58);
    left_.setJointValueTarget("left_wrist_1_joint", -0.585);
    left_.setJointValueTarget("left_wrist_2_joint", 1.57);
    left_.setJointValueTarget("left_wrist_3_joint", -1.57);

    // Specify a planner to be used for further planning
    left_.setPlannerId("RRTConnectkConfigDefault");
    left_.plan(plan);
    ROS_WARN("visualizing plan. STRG+C to interrupt.");
    moveit_simple_controller_manager::FollowJointTrajectoryControllerHandle handle_left(left_controller_,"follow_joint_trajectory");

    if (plan.trajectory_.joint_trajectory.joint_names.size() > 0){
        // check trajectory for collisions
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        planning_scene::PlanningScene planningScene(kinematic_model);
        bool isValid = planningScene.isPathValid(plan.start_state_,plan.trajectory_,"arms");
        if (!isValid){
            ROS_ERROR("Path is invalid. Execution aborted");
            return false;
        }
        else ROS_INFO("Checked path. Path is valid. Executing...");
    }
    bool success_left;
    /// Print out the joint trajectory infomation
    
    if (plan.trajectory_.joint_trajectory.joint_names.size() > 0){
        ROS_INFO("Trajectory sent to left arm");
        
        success_left = handle_left.sendTrajectory(plan.trajectory_);
    }
    if (plan.trajectory_.joint_trajectory.joint_names.size() > 0){
        success_left = handle_left.waitForExecution();
    }
    sleep(2);
    //left_.execute(plan);
    sleep(5);


    // ::::::: Run Experiments :::::::
    // variables
    double velocity = 0.005;

    // create ur_logger. Use this namespace
    std::vector<std::string> ur_namespaces;
    ur_namespaces.push_back("left");
    UR_Logger ur_logger(nh, ur_namespaces);

    // stop controller
    ros::ServiceClient left_srv_switch_controller = nh.serviceClient<controller_manager_msgs::SwitchController>("/left/controller_manager/switch_controller");
    controller_manager_msgs::SwitchController srv_req;
    srv_req.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
    srv_req.request.stop_controllers.push_back("/left/left_vel_based_pos_traj_controller");
    bool success = left_srv_switch_controller.call(srv_req);
    ROS_INFO("Stopping controller %s",success?"SUCCEDED":"FAILED");
    if (!success) return 0;
    
    srv_req.request.stop_controllers.clear();

    geometry_msgs::PoseStamped current_pose = left_.getCurrentPose();
    std::cout << "x: " << current_pose.pose.position.x << "\ty: " << current_pose.pose.position.y << "\tz:  " << current_pose.pose.position.z << std::endl;
    double radius = sqrt(current_pose.pose.position.x*current_pose.pose.position.x + current_pose.pose.position.y*current_pose.pose.position.y);
    std::cout << "RADIUS: " << radius << std::endl;
    double omega = velocity/radius;
    //double moving_time = moving_distance/velocity; //move 3cm, obstacle is around 1-2 cm away.

    // Output Message Speed = const
    trajectory_msgs::JointTrajectory joint_traj; //containing speed command

    // joint_traj.joint_names.push_back("left_shoulder_pan_joint");
    // joint_traj.joint_names.push_back("left_shoulder_lift_joint");
    // joint_traj.joint_names.push_back("left_elbow_joint");
    // joint_traj.joint_names.push_back("left_wrist_1_joint");
    // joint_traj.joint_names.push_back("left_wrist_2_joint");
    // joint_traj.joint_names.push_back("left_wrist_3_joint");

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

    ROS_INFO("trying to hold 30N");
    for(int i=0; i<joint_traj.joint_names.size(); ++i){
        ROS_INFO("Joint name  %s", joint_traj.joint_names[i].c_str());
    }
    while (ros::ok() && (stopwatch.elapsed().toSec()<20))
    {
        double res_force = sqrt(subscriber.last_wrench_msg_.Fx*subscriber.last_wrench_msg_.Fx 
                                +subscriber.last_wrench_msg_.Fy*subscriber.last_wrench_msg_.Fy);

        if (res_force < 30){
            // Removes the last element in the vector, effectively reducing the container size by one.
            joint_traj.points.pop_back();
            traj_point.velocities[0] = -omega;
            joint_traj.points.push_back(traj_point);

        }
        else if (res_force  < 40){
            joint_traj.points.pop_back();
            traj_point.velocities[0] = 0.0;
            joint_traj.points.push_back(traj_point);
        }
        else{
            joint_traj.points.pop_back();
            traj_point.velocities[0] = omega;
            joint_traj.points.push_back(traj_point);
        }

        joint_traj.header.stamp = ros::Time::now();
        left_speed_pub.publish(joint_traj);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("moving back");
    stopwatch.restart();

    while (ros::ok() && (stopwatch.elapsed().toSec()<10))
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


    ROS_INFO("moveing to start");
    left_.plan(plan);
    left_.execute(plan);


    ROS_INFO("finished. shutting down.");

    ros::shutdown();
    return 0;
}
