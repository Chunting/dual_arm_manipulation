# include "ur_logging/UrMessageListener.h"


//UR_Message_Listener::UR_Message_Listener(ros::NodeHandle& nh, std::string ur_namespace) : nh_(nh), ur_namespace_(ur_namespace){
UR_Message_Listener::UR_Message_Listener(ros::NodeHandle& nh, std::string ur_namespace) : nh_(nh), ur_namespace_(ur_namespace){
    wrench_sub_ = nh_.subscribe("/robotiq_force_torque_wrench", 1, &UR_Message_Listener::wrenchCallback, this);
    speed_traj_sub_ = nh_.subscribe(ur_namespace+"/ur_driver/joint_speed", 1, &UR_Message_Listener::speed_trajCallback, this);
    state_sub_ = nh_.subscribe("/joint_states", 1, &UR_Message_Listener::stateCallback, this);
    execTrajectorySub_ = nh_.subscribe<moveit_msgs::RobotTrajectory>("/execute_my_trajectory", 1, &UR_Message_Listener::trajectoryCallback, this);
    // It takes a long time to get message from this topic, about 0.5s
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(ur_namespace_+"/pose", 1, &UR_Message_Listener::poseCallback, this);
    newTrajectory = false;
    if (ur_namespace_.size() > 0){
        ur_prefix_ = ur_namespace_+"_";
    }
    ros::Subscriber sub1 = nh_.subscribe(ur_namespace+"/robotiq_force_torque_sensor",1, &UR_Message_Listener::reCallback, this);
    // ros::ServiceClient client = nh_.serviceClient<robotiq_force_torque_sensor::sensor_accessor>("robotiq_force_torque_sensor_acc");
    
    // robotiq_force_torque_sensor::sensor_accessor srv;

	int count = 0;
    ros::ok();
	// while (ros::ok())
	// {
	// 	// if(count == 10000000){
	// 	// 	srv.request.command = "SET ZRO";
	// 	// 	if(client.call(srv)){
	// 	// 		// ROS_INFO("ret: %s", srv.response.res.c_str());
	// 	// 	}
	// 	// 	count = 0;
	// 	// }
	// 	// ++count;
	// }

    
}

void UR_Message_Listener::wrenchCallback(const geometry_msgs::WrenchStamped::Ptr& msg){
    last_wrench_msg_ = *msg;
    // ROS_INFO("I heard last_wrench_msg_ : Frame_id [%s] Time [%f] FX[%f] FY[%f] FZ[%f] MX[%f] MY[%f] MZ[%f]",  
    //     last_wrench_msg_.header.frame_id.c_str(),   // robotiq_force_torque_frame_id
    //     last_wrench_msg_.header.stamp.toSec(),
    //     last_wrench_msg_.wrench.force.x, last_wrench_msg_.wrench.force.y, last_wrench_msg_.wrench.force.z, 
    //     last_wrench_msg_.wrench.torque.x, last_wrench_msg_.wrench.torque.y, last_wrench_msg_.wrench.torque.z);
}


void UR_Message_Listener::speed_trajCallback(const trajectory_msgs::JointTrajectory::Ptr& msg){
    last_speed_traj_msg_ = *msg;
}


void UR_Message_Listener::stateCallback(const sensor_msgs::JointState::Ptr& msg){
    std::string name = (*msg).name[0];
    if (name.compare(0,ur_prefix_.size(), ur_prefix_) == 0){    // "joint_states" receives messages of all robots. This is seperating the searched one from the others.
        last_state_msg_ = *msg;
    }
}
void UR_Message_Listener::reCallback(const robotiq_force_torque_sensor::ft_sensor::Ptr& msg)
{
    ROS_INFO("I heard: ");
    last_force_msg_ = *msg;
	ROS_INFO("I heard: FX[%f] FY[%f] FZ[%f] MX[%f] MY[%f] MZ[%f]", 
        last_force_msg_.Fx, last_force_msg_.Fy, last_force_msg_.Fz, 
        last_force_msg_.Mx, last_force_msg_.My, last_force_msg_.Mz);
}
void UR_Message_Listener::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    last_pose_msg_ = *msg;
    // ROS_INFO("\nframe_id: %s, %s  x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f\n", 
    //     last_pose_msg_.header.frame_id.c_str()
    //     ,ur_namespace_.c_str()
    //     ,last_pose_msg_.pose.position.x
    //     ,last_pose_msg_.pose.position.y
    //     ,last_pose_msg_.pose.position.z
    //     ,last_pose_msg_.pose.orientation.x
    //     ,last_pose_msg_.pose.orientation.y
    //     ,last_pose_msg_.pose.orientation.z
    //     ,last_pose_msg_.pose.orientation.w);
}

void UR_Message_Listener::trajectoryCallback(const moveit_msgs::RobotTrajectory::ConstPtr& msg){
    std::string left = "left_";
    std::string right = "right_";
    if (left.compare(0,ur_prefix_.size(), ur_prefix_) == 0  && (msg->joint_trajectory.points.size()>0) ) {    // "joint_states" receives messages of all robots. This is seperating the searched one from the others.
        last_trajectory_msg_ = *(msg.get());
        newTrajectory = true;
        // ROS_INFO("Header time  %f ", last_trajectory_msg_.joint_trajectory.header.stamp.toSec());
        // for (unsigned int i = 0; i < last_trajectory_msg_.joint_trajectory.points.size(); i++){
        //     ROS_INFO("Listening Points %d  %f ", i, last_trajectory_msg_.joint_trajectory.header.stamp.toSec()+last_trajectory_msg_.joint_trajectory.points[i].time_from_start.toSec());
        //     for (unsigned int a = 0; a < last_trajectory_msg_.joint_trajectory.points[i].positions.size(); a++){
        //         ROS_INFO("%s:\tpos %f\tvel %f", 
        //         last_trajectory_msg_.joint_trajectory.joint_names[a].c_str(), 
        //         last_trajectory_msg_.joint_trajectory.points[i].positions[a],
        //         last_trajectory_msg_.joint_trajectory.points[i].velocities[a]);
        //     }
        // }
    } else if(right.compare(0,ur_prefix_.size(), ur_prefix_) == 0  && (msg->joint_trajectory.points.size()>0) ){
        last_trajectory_msg_ = *(msg.get());
        newTrajectory = true;
    }
    
}
