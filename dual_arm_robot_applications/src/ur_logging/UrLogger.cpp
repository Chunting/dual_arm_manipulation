#include "ur_logging/UrLogger.h"

/* TODO
 * Position Message Data Line
 */

UR_Logger::UR_Logger(ros::NodeHandle &nh, std::vector<std::string> ur_namespaces) : nh_(nh) {
    for (int i = 0; i < ur_namespaces.size(); i++){
        ur_listeners_.push_back(new UR_Message_Listener(nh_, ur_namespaces[i]));
    }

    delimiter_ =',';

    ROS_INFO("Initialising UR Logger"); //it needs to be waited until msgs can be received
    sleep(1);
}


UR_Logger::~UR_Logger(){
    logfile_.close();   // close file when shutdown
    for (int i = 0; i < ur_listeners_.size(); i++){
        delete ur_listeners_[i];
    }
}


void UR_Logger::start(int log_rate){
    if (logfile_name_ == "" || logfile_name_command_ == ""){     //automatically generate a name if no name specified
        generate_logfile_name();
    }

    // write headline
    logfile_.open(logfile_name_.c_str(), std::ofstream::out | std::ofstream::trunc);   //generate new file from beginning
    if(logfile_.is_open()){
        for (int i = 0; i < ur_listeners_.size(); i++){
            logfile_ << headline(*ur_listeners_[i]);
            if (i < (ur_listeners_.size()-1)){
                logfile_ << delimiter_;
            }
        }
        logfile_ << std::endl;
        ROS_INFO("Writing log at %iHz to %s. Press Ctrl-C to stop.", log_rate, logfile_name_.c_str());
    }

    logfile_command_.open(logfile_name_command_.c_str(), std::ofstream::out | std::ofstream::trunc);   //generate new file from beginning
    if(logfile_command_.is_open()){
        logfile_command_ << "PC_time" << delimiter_ << "head_time";
        for (int i = 0; i < ur_listeners_.size(); i++){
            logfile_command_ << headline_command(*ur_listeners_[i]);
            // if (i < (ur_listeners_.size()-1)){
            //     logfile_command_ << delimiter_;
            // }
        }
        logfile_command_ << std::endl;
        ROS_INFO("Writing log at %iHz to %s. Press Ctrl-C to stop.", log_rate, logfile_name_command_.c_str());
    }
    stopwatch_.restart();
    double log_duration;
    log_duration = 1.0/log_rate;
    timer_ = nh_.createTimer(ros::Duration().fromSec(log_duration), &UR_Logger::logCallback, this);
}


void UR_Logger::stop(){
    ROS_INFO("Stopped logging");
    timer_.stop();
    logfile_.close();
    logfile_command_.close();
}


void UR_Logger::generate_logfile_name(){       //automatically generate a name
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [20];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime (buffer,20,"%Y_%m_%d_%H_%M%S", timeinfo);
    std::string log_suffix = buffer;
    logfile_name_="./src/dual_arm_manipulation/dataLog/ur_log_"+log_suffix+".csv";
    logfile_name_command_ = "./src/dual_arm_manipulation/dataLog/ur_command_"+log_suffix+".csv";
}


std::string UR_Logger::headline(UR_Message_Listener ur_listener){
    std::vector<std::string> joint_names;
    nh_.getParam(ur_listener.ur_namespace_+"/hardware_interface/joints", joint_names);
    std::cout << "namespace: " << ur_listener.ur_namespace_ << "/hardware_interface/joints" << std::endl;
    if (joint_names.size() < 6){
        ROS_ERROR("UR Logger: could not properly load joint names");
    }
    std::stringstream ss;

    // append time
    ss << "time";

    // append position state
    std::string state_pos_prefix = "state_pos_";
    //std::string state_pos_suffix = " [rad]";
    for (int i = 0; i < 6; i++ ){
        ss << delimiter_ << state_pos_prefix << ur_listener.last_state_msg_.name[i];// << state_pos_suffix;
    }

    // append state velocity
    std::string state_vel_prefix = "state_vel_";
    //std::string state_vel_suffix = " [rad/s]";
    for (int i = 0; i < 6; i++ ){
        ss << delimiter_ << state_vel_prefix << ur_listener.last_state_msg_.name[i];// << state_vel_suffix;
    }

    // append Cartesian state position
    std::string state_C_prefix = "state_pos_"+ur_listener.ur_namespace_+"_";
    //std::string wrench_force_suffix = " [N]";
    ss << delimiter_ << state_C_prefix << "x"
        << delimiter_ << state_C_prefix << "y"
        << delimiter_ << state_C_prefix << "z";
     // append Cartesian state rotation
    std::string state_R_prefix = "state_rot_"+ur_listener.ur_namespace_+"_";
    //std::string wrench_force_suffix = " [N]";
    ss << delimiter_ << state_R_prefix << "qx"
        << delimiter_ << state_R_prefix << "qy"
        << delimiter_ << state_R_prefix << "qz"
        << delimiter_ << state_R_prefix << "qw";
/*
    // append wrench
    std::string wrench_force_prefix = "tcp_wrench_force_";
    //std::string wrench_force_suffix = " [N]";
    ss << delimiter_ << wrench_force_prefix << "x"
        << delimiter_ << wrench_force_prefix << "y"
        << delimiter_ << wrench_force_prefix << "z";
    std::string wrench_torque_prefix = "tcp_wrench_torque_";
    //std::string wrench_torque_suffix = " [Nm]";
    ss << delimiter_ << wrench_torque_prefix << "x" 
        << delimiter_ << wrench_torque_prefix << "y" 
        << delimiter_ << wrench_torque_prefix << "z";

    // append target position
    std::string target_pos_prefix = "target_pos_";
    //std::string target_pos_suffix = " [rad]";
    for (int i = 0; i < 6; i++ ){
        ss << delimiter_ << target_pos_prefix << joint_names[i];// << target_pos_suffix;
    }

    // append target velocity
    std::string target_vel_prefix = "target_vel_";
    //std::string target_vel_suffix = " [rad/s]";
    for (int i = 0; i < 6; i++ ){
        ss << delimiter_ << target_vel_prefix << joint_names[i];// << target_vel_suffix;
    }
*/   
    return ss.str();
}


std::string UR_Logger::data_line(UR_Message_Listener ur_listener){

    std::ostringstream converter;    // stream used to convert numbers to string
    // append time
    converter << (stopwatch_.elapsed().toSec());

    // append position state
    //std::vector<double> state_pos = last_state_msg_.position;
    if (ur_listener.last_state_msg_.position.size() == 6){        
        for (int i = 0; i < 6; i++ ){
            converter << delimiter_ <<ur_listener.last_state_msg_.position[i]*(180/3.14159);
        }
    }
    else {
        converter << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_;
    }
    // append state velocity
    if (ur_listener.last_state_msg_.velocity.size() == 6){
        for (int i = 0; i < 6; i++ ){
            converter << delimiter_ << ur_listener.last_state_msg_.velocity[i]*(180/3.14159);
        }
    }
    else {
        converter << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_;
    }
    // append Cartesian position 
    
    converter << delimiter_ << ur_listener.last_tform_msg_.transform.translation.x
        << delimiter_ << ur_listener.last_tform_msg_.transform.translation.y
        << delimiter_ << ur_listener.last_tform_msg_.transform.translation.z;
    // append Cartesian rotation quaternion
    converter << delimiter_ << ur_listener.last_tform_msg_.transform.rotation.x
        << delimiter_ << ur_listener.last_tform_msg_.transform.rotation.y
        << delimiter_ << ur_listener.last_tform_msg_.transform.rotation.z
        << delimiter_ << ur_listener.last_tform_msg_.transform.rotation.w;
        

    /*
    // append tcp wrench force
    converter << delimiter_ << ur_listener.last_wrench_msg_.wrench.force.x
        << delimiter_ << ur_listener.last_wrench_msg_.wrench.force.y
        << delimiter_ << ur_listener.last_wrench_msg_.wrench.force.z;
    // tcp wrench torque
    converter << delimiter_ << ur_listener.last_wrench_msg_.wrench.torque.x
        << delimiter_ << ur_listener.last_wrench_msg_.wrench.torque.y
        << delimiter_ << ur_listener.last_wrench_msg_.wrench.torque.z;

    // append target position
    if (ur_listener.last_trajectory_msg_.joint_trajectory.points.size() > 0) {
        for (unsigned int i = 0; i < ur_listener.last_trajectory_msg_.joint_trajectory.points.size(); i++){
            for (unsigned int a = 0; a < ur_listener.last_trajectory_msg_.joint_trajectory.points[i].positions.size(); a++){
                ROS_INFO("%s:\tpos %f\tvel %f", 
                ur_listener.last_trajectory_msg_.joint_trajectory.joint_names[a].c_str(), 
                ur_listener.last_trajectory_msg_.joint_trajectory.points[i].positions[a],
                ur_listener.last_trajectory_msg_.joint_trajectory.points[i].velocities[a]);
            }
        }
    }else {
        converter << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_;
    }

    // // append target velocity
    // if (ur_listener.last_speed_traj_msg_.points.size()==1){
    //     trajectory_msgs::JointTrajectoryPoint traj_point = ur_listener.last_speed_traj_msg_.points[0];
    //     for (int i = 0; i < 6; i++ ){
    //         converter << delimiter_ << traj_point.velocities[i];
    //     }
    // }
    // else {
    //     converter << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_;
    // }
    */
    
    return converter.str();
}


std::string UR_Logger::headline_command(UR_Message_Listener ur_listener){
    std::vector<std::string> joint_names;
    nh_.getParam(ur_listener.ur_namespace_+"/hardware_interface/joints", joint_names);
    if (joint_names.size() < 6){
        ROS_ERROR("UR Logger: could not properly load joint names");
    }
    std::stringstream ss;
    
    // append time
    // ss << "PC_time" << delimiter_ << "head_time";

    // append position command
    std::string command_pos_prefix = "command_pos_";
    for (int i = 0; i < ur_listener.last_state_msg_.name.size(); i++ ){
        ss << delimiter_ << command_pos_prefix << joint_names[i];
    }

    // append command velocity
    std::string command_vel_prefix = "command_vel_";
    for (int i = 0; i < ur_listener.last_state_msg_.name.size(); i++ ){
        ss << delimiter_ << command_vel_prefix << joint_names[i];
    }

    // std::cout << ss.str() << std::endl;
    
    return ss.str();
}
std::string UR_Logger::data_line_command(UR_Message_Listener ur_listener){
    std::ostringstream converter;    // stream used to convert numbers to string
    // append target position
    for (unsigned int i = 0; i < ur_listener.last_trajectory_msg_.joint_trajectory.points.size(); i++){
        // append time
        converter << (stopwatch_.elapsed().toSec()) << delimiter_ << ur_listener.last_trajectory_msg_.joint_trajectory.points[i].time_from_start.toSec();
        for (unsigned int a = 0; a < 6; a++){
            converter << delimiter_ << ur_listener.last_trajectory_msg_.joint_trajectory.points[i].positions[a];  
        }
        for (unsigned int a = 0; a < 6; a++){
            converter << delimiter_ << ur_listener.last_trajectory_msg_.joint_trajectory.points[i].velocities[a];  
        }
        for (unsigned int a = 6; a < ur_listener.last_trajectory_msg_.joint_trajectory.points[i].positions.size(); a++){
            converter << delimiter_ << ur_listener.last_trajectory_msg_.joint_trajectory.points[i].positions[a];  
        }
        for (unsigned int a = 6; a < ur_listener.last_trajectory_msg_.joint_trajectory.points[i].positions.size(); a++){
            converter << delimiter_ << ur_listener.last_trajectory_msg_.joint_trajectory.points[i].velocities[a];  
        }
        converter << std::endl;
    }  
    
    return converter.str();
}


void UR_Logger::logCallback(const ros::TimerEvent&){
    for(int i = 0; i < ur_listeners_.size(); i++){
        logfile_ << data_line(*ur_listeners_[i]);
        if (i < (ur_listeners_.size()-1)){
            logfile_ << delimiter_;
        }
    }
    logfile_ << std::endl;

    for(int i = 0; i < ur_listeners_.size(); i++){
        logfile_command_ << data_line_command(*ur_listeners_[i]);
        // if (i < (ur_listeners_.size()-1)){
        //     logfile_command_ << delimiter_;
        // }
    }
    //logfile_command_ << std::endl;
    
}

/* Simple Log-Node
int main(int argc, char **argv){
    ros::init(argc, argv, "ur_logger");
    ros::NodeHandle nh;

    ros::AsyncSpinner asyncSpinner(1);
    asyncSpinner.start();

    std::vector<std::string> ur_namespaces;
    ur_namespaces.push_back("");
    UrLogger ur_logger(nh, ur_namespaces);

    ur_logger.start(4);
    ros::Duration(3).sleep();
    ur_logger.stop();

    ros::shutdown();
    return 0;
}
*/
