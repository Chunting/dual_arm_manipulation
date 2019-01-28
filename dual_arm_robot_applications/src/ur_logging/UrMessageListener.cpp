# include "ur_logging/UrMessageListener.h"


//UR_Message_Listener::UR_Message_Listener(ros::NodeHandle& nh, std::string ur_namespace) : nh_(nh), ur_namespace_(ur_namespace){
UR_Message_Listener::UR_Message_Listener(ros::NodeHandle& nh, std::string ur_namespace) : nh_(nh), ur_namespace_(ur_namespace){
    wrench_sub_ = nh_.subscribe(ur_namespace+"/tcp_wrench", 1, &UR_Message_Listener::wrenchCallback, this);
    speed_traj_sub_ = nh_.subscribe(ur_namespace+"/ur_driver/joint_speed", 1, &UR_Message_Listener::speed_trajCallback, this);
    state_sub_ = nh_.subscribe("/joint_states", 1, &UR_Message_Listener::stateCallback, this);
    execTrajectorySub_ = nh_.subscribe<moveit_msgs::RobotTrajectory>("/execute_my_trajectory", 1, &UR_Message_Listener::trajectoryCallback, this);
    
    //m_rawTFLogLatest.fromNSec(0);
    pose_sub_ = nh_.subscribe("/tf", 1, &UR_Message_Listener::tfCallback, this);
    m_tf = new tf::Transformer();

    if (ur_namespace_.size() > 0){
        ur_prefix_ = ur_namespace_+"_";
    }
}

void UR_Message_Listener::wrenchCallback(const geometry_msgs::WrenchStamped::Ptr& msg){
    last_wrench_msg_ = *msg;
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

void UR_Message_Listener::tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg){
    // ROS_INFO("Transforms size : %d", msg->transforms.size());
    std::string source_frameid = "world";
    std::string target_frameid = ur_prefix_ + "tool0";
    tf::TransformListener tfListener;
    // Wait for up to one second for the first transforms to become avaiable. 
    tfListener.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.0));
    tf::StampedTransform transform;
    try{
        tfListener.lookupTransform(source_frameid, target_frameid, ros::Time(), transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }
    last_tform_msg_.transform.translation.x = transform.getOrigin().x();
    last_tform_msg_.transform.translation.y = transform.getOrigin().y();
    last_tform_msg_.transform.translation.z = transform.getOrigin().z();
    last_tform_msg_.transform.rotation.x = transform.getRotation().x();
    last_tform_msg_.transform.rotation.y = transform.getRotation().y();
    last_tform_msg_.transform.rotation.z = transform.getRotation().z();
    last_tform_msg_.transform.rotation.w = transform.getRotation().w();
    // std::cout.precision(3);
    // std::cout.setf(std::ios::fixed,std::ios::floatfield);
    // std::cout << "At time " << last_transform_.stamp_.toSec() << "  " << source_frameid << "   " << target_frameid << std::endl;
    // double yaw, pitch, roll;
    // last_transform_.getBasis().getRPY(roll, pitch, yaw);
    // tf::Quaternion q = last_transform_.getRotation();
    // tf::Vector3 v = last_transform_.getOrigin();
    // std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
    // std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", " 
    //           << q.getZ() << ", " << q.getW() << "]" << std::endl
    //           << "            in RPY (radian) [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl
    //           << "            in RPY (degree) [" <<  roll*180.0/M_PI << ", " << pitch*180.0/M_PI << ", " << yaw*180.0/M_PI << "]" << std::endl;
 

    // for(std::size_t i = 0; i < msg->transforms.size(); ++i)
	// {
    //     if(msg->transforms[i].header.frame_id.compare(0,frame_id.size(),frame_id)==0 ){
    //         //&& msg->transforms[i].child_frame_id.compare(0,child_frame_id.size(), child_frame_id)==0){
    //             last_tform_msg_ = msg->transforms[i];

    //         // ROS_INFO("I heard %d: child_frame %s \t frame_id: %s\n x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f ", i,
    //         //         last_tform_msg_.child_frame_id.c_str(), 
    //         //         last_tform_msg_.header.frame_id.c_str(),  
    //         //         last_tform_msg_.transform.translation.x,
    //         //         last_tform_msg_.transform.translation.y,
    //         //         last_tform_msg_.transform.translation.z,
    //         //         last_tform_msg_.transform.rotation.x,
    //         //         last_tform_msg_.transform.rotation.y,
    //         //         last_tform_msg_.transform.rotation.z,
    //         //         last_tform_msg_.transform.rotation.w );
    //     } 
    // }
}

void UR_Message_Listener::trajectoryCallback(const moveit_msgs::RobotTrajectory::ConstPtr& msg){
    last_trajectory_msg_ = *(msg.get());
    if (last_trajectory_msg_.joint_trajectory.points.size() == 0)  return;
    for (unsigned int i = 0; i < last_trajectory_msg_.joint_trajectory.points.size(); i++){
        ROS_INFO("Points %d", i);
        for (unsigned int a = 0; a < last_trajectory_msg_.joint_trajectory.points[i].positions.size(); a++){
            ROS_INFO("%s:\tpos %f\tvel %f", 
            last_trajectory_msg_.joint_trajectory.joint_names[a].c_str(), 
            last_trajectory_msg_.joint_trajectory.points[i].positions[a],
            last_trajectory_msg_.joint_trajectory.points[i].velocities[a]);
        }
    }
}
