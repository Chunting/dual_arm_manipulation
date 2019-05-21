#include "ur_logging/UrLogger.h"

/* TODO
 * Position Message Data Line
 */
const float rot2deg = 1; //180/3.14159;

UR_Logger::UR_Logger(ros::NodeHandle &nh, std::vector<std::string> &ur_namespaces) : nh_(nh)
{
    logfolder_name_ = generate_logfolder();
    for (int i = 0; i < ur_namespaces.size(); i++)
    {
        ur_listeners_.push_back(std::make_shared<UR_Message_Listener> (nh_,  ur_namespaces[i], logfolder_name_));
    }

    delimiter_ = ',';
    ROS_INFO("Initialising UR Logger"); //it needs to be waited until msgs can be received
    sleep(1);
}

UR_Logger::~UR_Logger()
{
    ur_listeners_.clear();
}

void UR_Logger::start(int log_rate)
{
    for (int i = 0; i < ur_listeners_.size(); i++)
    {
        ur_listeners_[i]->start(log_rate);
    }
    if (ft_sensor_logfile_name_ == "" || robot_cart_pose_logfile_name_ == "")
    {
        generate_logfile_name();
    }

    if(!create_ft_sensor_logfile()){
        return;
    }
    if(!create_robot_cart_pose_logfile()){
        return;
    }

    stopwatch_.restart();
    double log_duration;
    log_duration = 1.0 / log_rate;
    timer_ = nh_.createTimer(ros::Duration().fromSec(log_duration), &UR_Logger::logCallback, this);
}
void UR_Logger::stop()
{
    ROS_INFO("Stopped logging");
    timer_.stop();
    ft_sensor_logfile_stream_.close();
    robot_cart_pose_logfile_stream_.close();
}

void UR_Logger::generate_logfile_name()
{ 
    ft_sensor_logfile_name_ = logfolder_name_ + "ft_sensor_log.csv";
    robot_cart_pose_logfile_name_ = logfolder_name_ + "robot_cart_pose_log.csv";
}
std::string UR_Logger::ft_sensor_headline(UR_Message_Listener &ur_listener)
{
    std::stringstream ss;
    ss << ur_listener.ur_namespace_ + "_Time" 
       << delimiter_ << ur_listener.ur_namespace_ + "_Fx"
       << delimiter_ << ur_listener.ur_namespace_ + "_Fy"
       << delimiter_ << ur_listener.ur_namespace_ + "_Fz"
       << delimiter_ << ur_listener.ur_namespace_ + "_Tx"
       << delimiter_ << ur_listener.ur_namespace_ + "_Ty"
       << delimiter_ << ur_listener.ur_namespace_ + "_Tz"
       << delimiter_ << ur_listener.ur_namespace_ + "_Fs";
    return ss.str();
}
std::string UR_Logger::ft_sensor_headline()
{
    std::stringstream ss;
    for (int i = 0; i < ur_listeners_.size(); i++)
    {
        ss << ft_sensor_headline(*ur_listeners_[i]) << delimiter_;
    }
    ss  <<"Relative_Fx" << delimiter_ 
        <<"Relative_Fy" << delimiter_ 
        <<"Relative_Fz" << delimiter_ 
        <<"Relative_Tx" << delimiter_ 
        <<"Relative_Ty" << delimiter_ 
        <<"Relative_Tz" << std::endl;
    return ss.str();
}

bool UR_Logger::create_ft_sensor_logfile()
{
    if(ft_sensor_logfile_name_.empty()){
        ROS_ERROR("Failed to create robotiq ft sensor logfile!");
        return false;
    }
    ft_sensor_logfile_stream_.open(ft_sensor_logfile_name_.c_str(), std::ofstream::out | std::ofstream::trunc); //generate new file from beginning
    if (ft_sensor_logfile_stream_.is_open())
    {
        ft_sensor_logfile_stream_ << ft_sensor_headline();
    }else{
        ROS_ERROR("UrLogger failed! log file %s", ft_sensor_logfile_name_.c_str());
        return false;
    }
    return true;

}
bool UR_Logger::create_robot_cart_pose_logfile()
{
    if(robot_cart_pose_logfile_name_.empty()){
        ROS_ERROR("Failed to create robot cart pose logfile!");
        return false;
    }
    robot_cart_pose_logfile_stream_.open(robot_cart_pose_logfile_name_.c_str(), std::ofstream::out | std::ofstream::trunc); //generate new file from beginning
    if (robot_cart_pose_logfile_stream_.is_open())
    {
        robot_cart_pose_logfile_stream_ << robot_cart_pose_headline();
    }else{
        ROS_ERROR("UrLogger failed! log file %s", ft_sensor_logfile_name_.c_str());
        return false;
    }
    return true;

}
std::string UR_Logger::robot_cart_pose_headline(UR_Message_Listener &ur_listener)
{
    std::stringstream ss;
    ss << ur_listener.ur_namespace_ + "_state_time"
        << delimiter_ << ur_listener.ur_namespace_ + "_state_x" 
        << delimiter_ << ur_listener.ur_namespace_ + "_state_y"
        << delimiter_ << ur_listener.ur_namespace_ + "_state_z" 
        << delimiter_ << ur_listener.ur_namespace_ + "_state_qx"
        << delimiter_ << ur_listener.ur_namespace_ + "_state_qy" 
        << delimiter_ << ur_listener.ur_namespace_ + "_state_qz"
        << delimiter_ << ur_listener.ur_namespace_ + "_state_qw"
        << delimiter_ << ur_listener.ur_namespace_ + "_cmd_time"
        << delimiter_ << ur_listener.ur_namespace_ + "_time_from_start"
        << delimiter_ << ur_listener.ur_namespace_ + "_cmd_x" 
        << delimiter_ << ur_listener.ur_namespace_ + "_cmd_y"
        << delimiter_ << ur_listener.ur_namespace_ + "_cmd_z" 
        << delimiter_ << ur_listener.ur_namespace_ + "_cmd_qx"
        << delimiter_ << ur_listener.ur_namespace_ + "_cmd_qy" 
        << delimiter_ << ur_listener.ur_namespace_ + "_cmd_qz"
        << delimiter_ << ur_listener.ur_namespace_ + "_cmd_qw";
    return ss.str();
}
//@TODO by Chunting, Add the absolute error and relative error
std::string UR_Logger::robot_cart_pose_headline()
{
    std::stringstream ss;
    for (int i = 0; i < ur_listeners_.size(); i++)
    {
        ss << robot_cart_pose_headline(*ur_listeners_[i]) << delimiter_;
    }
    ss << "offset_x" << delimiter_ 
       << "offset_y" << delimiter_ 
       << "offset_z" << delimiter_ 
       << "offset_qx" << delimiter_ 
       << "offset_qy" << delimiter_ 
       << "offset_qz" << delimiter_ 
       << "offset_qw" << std::endl;
    return ss.str();
}


// @TODO by Chunting, transform force data into world coordinate system
std::string UR_Logger::ft_sensor_dataline(UR_Message_Listener &ur_listener)
{
    std::ostringstream converter; // stream used to convert numbers to string
    converter << ur_listener.last_wrench_msg_.header.stamp.toSec()-ur_listener.start_time_        
              << delimiter_ << ur_listener.last_wrench_msg_.wrench.force.x
              << delimiter_ << ur_listener.last_wrench_msg_.wrench.force.y
              << delimiter_ << ur_listener.last_wrench_msg_.wrench.force.z
              << delimiter_ << ur_listener.last_wrench_msg_.wrench.torque.x
              << delimiter_ << ur_listener.last_wrench_msg_.wrench.torque.y
              << delimiter_ << ur_listener.last_wrench_msg_.wrench.torque.z
              << delimiter_<< sqrt(ur_listener.last_wrench_msg_.wrench.force.x * ur_listener.last_wrench_msg_.wrench.force.x 
                                    + ur_listener.last_wrench_msg_.wrench.force.y * ur_listener.last_wrench_msg_.wrench.force.y 
                                    + ur_listener.last_wrench_msg_.wrench.force.z * ur_listener.last_wrench_msg_.wrench.force.z);    
    return converter.str();
}
std::string UR_Logger::ft_sensor_dataline()
{
    std::ostringstream converter; // stream used to convert numbers to string
    for (int i = 0; i < ur_listeners_.size(); i++)
    {
       converter << ft_sensor_dataline(*ur_listeners_[i]) << delimiter_;
    }
    converter << ur_listeners_[0]->last_wrench_msg_.wrench.force.x - ur_listeners_[1]->last_wrench_msg_.wrench.force.x
              << delimiter_ << ur_listeners_[0]->last_wrench_msg_.wrench.force.y - ur_listeners_[1]->last_wrench_msg_.wrench.force.y
              << delimiter_ << ur_listeners_[0]->last_wrench_msg_.wrench.force.z - ur_listeners_[1]->last_wrench_msg_.wrench.force.z
              << delimiter_ << ur_listeners_[0]->last_wrench_msg_.wrench.torque.x - ur_listeners_[1]->last_wrench_msg_.wrench.torque.x
              << delimiter_ << ur_listeners_[0]->last_wrench_msg_.wrench.torque.y - ur_listeners_[1]->last_wrench_msg_.wrench.torque.y
              << delimiter_ << ur_listeners_[0]->last_wrench_msg_.wrench.torque.z - ur_listeners_[1]->last_wrench_msg_.wrench.torque.z
              << std::endl;
    
    return converter.str();
}
// @TODO by Chunting, transform force data into world coordinate system
std::string UR_Logger::robot_cart_pose_dataline(UR_Message_Listener &ur_listener)
{
    std::ostringstream converter; // stream used to convert numbers to string
    converter<< ur_listener.last_cart_pose_state_msg_.header.stamp.toSec() - ur_listener.start_time_
                    << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.position.x
                    << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.position.y
                    << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.position.z
                    << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.orientation.x
                    << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.orientation.y
                    << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.orientation.z
                    << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.orientation.w;
    if( ur_listener.last_cart_pose_cmd_msg_.header.stamp.toSec() > ur_listener.pre_cmd_time_ 
            && ur_listener.last_joint_traj_point_cmd_msg_.positions.size()>0){
        converter << ur_listener.last_cart_pose_cmd_msg_.header.stamp.toSec() - ur_listener.start_time_ 
                    << delimiter_ << ur_listener.last_joint_traj_point_cmd_msg_.time_from_start
                    << delimiter_ << ur_listener.last_cart_pose_cmd_msg_.pose.position.x
                    << delimiter_ << ur_listener.last_cart_pose_cmd_msg_.pose.position.y
                    << delimiter_ << ur_listener.last_cart_pose_cmd_msg_.pose.position.z
                    << delimiter_ << ur_listener.last_cart_pose_cmd_msg_.pose.orientation.x
                    << delimiter_ << ur_listener.last_cart_pose_cmd_msg_.pose.orientation.y
                    << delimiter_ << ur_listener.last_cart_pose_cmd_msg_.pose.orientation.z
                    << delimiter_ << ur_listener.last_cart_pose_cmd_msg_.pose.orientation.w;
        ur_listener.pre_cmd_time_ = ur_listener.last_cart_pose_cmd_msg_.header.stamp.toSec();
    }else{
        converter << ur_listener.last_cart_pose_state_msg_.header.stamp.toSec() - ur_listener.start_time_
                  << delimiter_ << 0.0
                  << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.position.x
                  << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.position.y
                  << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.position.z
                  << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.orientation.x
                  << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.orientation.y
                  << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.orientation.z
                  << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.orientation.w;
    }
        


    //for(int i = 0; i < ur_listeners_.size(); i++){
    if (ur_listeners_[0]->newTrajectory)
        robot_cart_pose_logfile_stream_ << data_line_command(*ur_listeners_[0]);
    //}
    return converter.str();
}
std::string UR_Logger::robot_cart_pose_dataline()
{
    std::ostringstream converter; // stream used to convert numbers to string
    for (int i = 0; i < ur_listeners_.size(); i++)
    {
       converter << robot_cart_pose_dataline(*ur_listeners_[i]) << delimiter_;
    }
    converter << ur_listeners_[0]->last_offset_pose_state_msg_.position.x
              << delimiter_ << ur_listeners_[0]->last_offset_pose_state_msg_.position.y
              << delimiter_ << ur_listeners_[0]->last_offset_pose_state_msg_.position.z
              << delimiter_ << ur_listeners_[0]->last_offset_pose_state_msg_.orientation.x
              << delimiter_ << ur_listeners_[0]->last_offset_pose_state_msg_.orientation.y
              << delimiter_ << ur_listeners_[0]->last_offset_pose_state_msg_.orientation.z
              << delimiter_ << ur_listeners_[0]->last_offset_pose_state_msg_.orientation.w;
    converter << std::endl;
    
    return converter.str();
}
void UR_Logger::logCallback(const ros::TimerEvent &)
{

    ft_sensor_logfile_stream_ << ft_sensor_dataline();
    robot_cart_pose_logfile_stream_ << robot_cart_pose_dataline();
}
std::string UR_Logger::headline(UR_Message_Listener &ur_listener)
{
    std::vector<std::string> joint_names;
    nh_.getParam(ur_listener.ur_namespace_ + "/hardware_interface/joints", joint_names);
    std::cout << "namespace: " << ur_listener.ur_namespace_ << "/hardware_interface/joints" << std::endl;
    if (joint_names.size() < 6)
    {
        ROS_ERROR("UR Logger: could not properly load joint names");
    }
    std::stringstream ss;

    // append time
    ss << "Time";
 
    // append position state
    for (int i = 0; i < 6; i++)
    {
        ss << delimiter_ << joint_names[i] + "_state_pos"; 
    }
    for (int i = 0; i < 6; i++)
    {
        ss << delimiter_ << joint_names[i] + "_state_vel";
    }
    std::string cart_pose_state_prefix = ur_listener.ur_namespace_ + "_cart_pose_state_";
    std::string cart_vel_state_prefix = ur_listener.ur_namespace_ + "_cart_vel_state_";
    std::string wrench_prefix = ur_listener.ur_namespace_ + "_wrench_";
    ss << delimiter_ << cart_pose_state_prefix << "x"
       << delimiter_ << cart_pose_state_prefix << "y"
       << delimiter_ << cart_pose_state_prefix << "z"
       << delimiter_ << cart_pose_state_prefix << "qx"
       << delimiter_ << cart_pose_state_prefix << "qy"
       << delimiter_ << cart_pose_state_prefix << "qz"
       << delimiter_ << cart_pose_state_prefix << "qw"
       << delimiter_ << cart_vel_state_prefix << "vx"
       << delimiter_ << cart_vel_state_prefix << "vy"
       << delimiter_ << cart_vel_state_prefix << "vz"
       << delimiter_ << cart_vel_state_prefix << "ax"
       << delimiter_ << cart_vel_state_prefix << "ay"
       << delimiter_ << cart_vel_state_prefix << "az"
       << delimiter_ << wrench_prefix << "Fx"
       << delimiter_ << wrench_prefix << "Fy"
       << delimiter_ << wrench_prefix << "Fz"
       << delimiter_ << wrench_prefix << "Fs"
       << delimiter_ << wrench_prefix << "Tx"
       << delimiter_ << wrench_prefix << "Ty"
       << delimiter_ << wrench_prefix << "Tz"
       << delimiter_ << "offset_x"
       << delimiter_ << "offset_y"
       << delimiter_ << "offset_z";

    return ss.str();
}
std::string UR_Logger::data_line(UR_Message_Listener &ur_listener)
{

    std::ostringstream converter; // stream used to convert numbers to string
    converter << (stopwatch_.elapsed().toSec());

    // append joint position state
    if (ur_listener.last_joint_state_msg_.position.size() == 6)
    {
        for (int i = 0; i < 6; i++)
        {
            converter << delimiter_ << ur_listener.last_joint_state_msg_.position[i] * rot2deg;
        }
    }
    else
    {
        converter << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_;
    }
    // append joint velocity
    if (ur_listener.last_joint_state_msg_.velocity.size() == 6)
    {
        for (int i = 0; i < 6; i++)
        {
            converter << delimiter_ << ur_listener.last_joint_state_msg_.velocity[i] * rot2deg;
           
        }
    }
    else
    {
        converter << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_;
    }

    // append Cartesian position
    converter << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.position.x
              << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.position.y
              << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.position.z;
    // append Cartesian rotation quaternion
    converter << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.orientation.x
              << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.orientation.y
              << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.orientation.z
              << delimiter_ << ur_listener.last_cart_pose_state_msg_.pose.orientation.w;

    // append measured Cartesian speed
    converter << delimiter_ << ur_listener.last_cart_vel_state_msg_.twist.linear.x
              << delimiter_ << ur_listener.last_cart_vel_state_msg_.twist.linear.y
              << delimiter_ << ur_listener.last_cart_vel_state_msg_.twist.linear.z
              << delimiter_ << ur_listener.last_cart_vel_state_msg_.twist.angular.x
              << delimiter_ << ur_listener.last_cart_vel_state_msg_.twist.angular.y
              << delimiter_ << ur_listener.last_cart_vel_state_msg_.twist.angular.z;

    // // append tcp wrench force
    converter << delimiter_ << ur_listener.last_wrench_msg_.wrench.force.x
              << delimiter_ << ur_listener.last_wrench_msg_.wrench.force.y
              << delimiter_ << ur_listener.last_wrench_msg_.wrench.force.z
              << delimiter_<< sqrt(ur_listener.last_wrench_msg_.wrench.force.x * ur_listener.last_wrench_msg_.wrench.force.x + ur_listener.last_wrench_msg_.wrench.force.y * ur_listener.last_wrench_msg_.wrench.force.y + ur_listener.last_wrench_msg_.wrench.force.z * ur_listener.last_wrench_msg_.wrench.force.z);
    // tcp wrench torque
    converter << delimiter_ << ur_listener.last_wrench_msg_.wrench.torque.x
              << delimiter_ << ur_listener.last_wrench_msg_.wrench.torque.y
              << delimiter_ << ur_listener.last_wrench_msg_.wrench.torque.z;

    converter << delimiter_ << ur_listener.last_offset_pose_state_msg_.position.x
              << delimiter_ << ur_listener.last_offset_pose_state_msg_.position.y
              << delimiter_ << ur_listener.last_offset_pose_state_msg_.position.z;

    return converter.str();
}

std::string UR_Logger::headline_command(UR_Message_Listener &ur_listener)
{
    std::vector<std::string> joint_names;
    nh_.getParam(ur_listener.ur_namespace_ + "/hardware_interface/joints", joint_names);
    if (joint_names.size() < 6)
    {
        ROS_ERROR("UR Logger: could not properly load joint names");
    }
    std::stringstream ss;

    // append time
    ss << "PC_time" << delimiter_ << "time_from_start";

    // append joint command
    for (int i = 0; i < ur_listener.last_joint_state_msg_.name.size(); i++)
    {
        ss << delimiter_ << joint_names[i] + "_cmd_pos";
    }

    for (int i = 0; i < ur_listener.last_joint_state_msg_.name.size(); i++)
    {
        ss << delimiter_ << joint_names[i] + "_cmd_vel";
    }
    ss  << delimiter_ << ur_listener.ur_namespace_ + "_cart_pose_cmd_x" 
        << delimiter_ << ur_listener.ur_namespace_ + "_cart_pose_cmd_y"
        << delimiter_ << ur_listener.ur_namespace_ + "_cart_pose_cmd_z"
        << delimiter_ << ur_listener.ur_namespace_ + "_cart_pose_cmd_qx"
        << delimiter_ << ur_listener.ur_namespace_ + "_cart_pose_cmd_qy" 
        << delimiter_ << ur_listener.ur_namespace_ + "_cart_pose_cmd_qz"
        << delimiter_ << ur_listener.ur_namespace_ + "_cart_pose_cmd_qw" << "\n";
    return ss.str();
}
std::string UR_Logger::data_line_command(UR_Message_Listener &ur_listener)
{
    std::ostringstream converter; // stream used to convert numbers to string
    //  Only record the new data
    if (ur_listener.newTrajectory == true)
    {
        for (unsigned int i = 0; i < ur_listener.last_robot_traj_cmd_msg_.joint_trajectory.points.size(); i++)
        {
            // append time      PCtime+time_from_start
            // It waits for 1 sec after pulish the trojectory
            double timeStamp = (stopwatch_.elapsed().toSec()) + ur_listener.last_robot_traj_cmd_msg_.joint_trajectory.points[i].time_from_start.toSec() + 1;
            converter << timeStamp << delimiter_ << ur_listener.last_robot_traj_cmd_msg_.joint_trajectory.points[i].time_from_start.toSec();
            int jn = ur_listener.last_robot_traj_cmd_msg_.joint_trajectory.joint_names.size();
            // joint position of left arm
            if (jn == 12)
            {
                for (unsigned int a = 0; a < 6; a++)
                {
                    converter << delimiter_ << ur_listener.last_robot_traj_cmd_msg_.joint_trajectory.points[i].positions[a];
                }
                // joint velocity of left arm
                for (unsigned int a = 0; a < 6; a++)
                {
                    converter << delimiter_ << ur_listener.last_robot_traj_cmd_msg_.joint_trajectory.points[i].velocities[a];
                }
                // joint position of right arm
                for (unsigned int a = 6; a < ur_listener.last_robot_traj_cmd_msg_.joint_trajectory.points[i].positions.size(); a++)
                {
                    converter << delimiter_ << ur_listener.last_robot_traj_cmd_msg_.joint_trajectory.points[i].positions[a];
                }
                // joint velocity of right arm
                for (unsigned int a = 6; a < ur_listener.last_robot_traj_cmd_msg_.joint_trajectory.points[i].positions.size(); a++)
                {
                    converter << delimiter_ << ur_listener.last_robot_traj_cmd_msg_.joint_trajectory.points[i].velocities[a];
                }
            }
            else if (jn == 6)
            {
                std::string jointname = ur_listener.last_robot_traj_cmd_msg_.joint_trajectory.joint_names[0];
                std::string right = "right_";
                if (jointname.compare(0, right.size(), right) == 0)
                {
                    converter << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_;
                    converter << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_;
                    for (unsigned int a = 0; a < 6; a++)
                    {
                        converter << delimiter_ << ur_listener.last_robot_traj_cmd_msg_.joint_trajectory.points[i].positions[a];
                    }
                    // joint velocity of right arm
                    for (unsigned int a = 0; a < 6; a++)
                    {
                        converter << delimiter_ << ur_listener.last_robot_traj_cmd_msg_.joint_trajectory.points[i].velocities[a];
                    }
                }
                else
                {
                    for (unsigned int a = 0; a < 6; a++)
                    {
                        converter << delimiter_ << ur_listener.last_robot_traj_cmd_msg_.joint_trajectory.points[i].positions[a];
                    }
                    // joint velocity of left arm
                    for (unsigned int a = 0; a < 6; a++)
                    {
                        converter << delimiter_ << ur_listener.last_robot_traj_cmd_msg_.joint_trajectory.points[i].velocities[a];
                    }
                    converter << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_;
                    converter << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_;
                }
            }
            converter << std::endl;
        }
        ur_listener.last_robot_traj_cmd_msg_.joint_trajectory.points.clear();
        ur_listener.newTrajectory = false;
    }

    return converter.str();
}

std::string UR_Logger::generate_logfolder(){
    // Creating a recording directory
  std::string recPath_ = "/home/chunting/catkin_ws/src/dual_arm_manipulation/dataLog/";
  time_t rawtime;
  struct tm *timeinfo;
  char buffer[20];

  time(&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer, 80, "%Y_%m_%d_%H_%M", timeinfo);
  recPath_ = recPath_ + std::string(buffer) + '/';
  const int folder = mkdir(recPath_.c_str(), 0777);
  if(folder == -1){
      ROS_ERROR("Failed to creat log folder %s", recPath_.c_str());
  }
  return recPath_;
}

// void UR_Logger::logCallback(const ros::TimerEvent &)
// {
//     for (int i = 0; i < ur_listeners_.size(); i++)
//     {
//         ur_listeners_[i]->write_logfile();
//         ft_sensor_logfile_stream_ << data_line(*ur_listeners_[i]);
//         if (i < (ur_listeners_.size() - 1))
//         {
//             ft_sensor_logfile_stream_ << delimiter_;
//         }
//     }
//     ft_sensor_logfile_stream_ << std::endl;

//     //for(int i = 0; i < ur_listeners_.size(); i++){
//     if (ur_listeners_[0]->newTrajectory)
//         robot_cart_pose_logfile_stream_ << data_line_command(*ur_listeners_[0]);
//     //}
// }



// /* Simple Log-Node */
// int main(int argc, char **argv){
//     ros::init(argc, argv, "ur_logger");
//     ros::NodeHandle nh;
//     ros::AsyncSpinner asyncSpinner(2);
//     asyncSpinner.start();
//     ros::Rate loop_rate_(100);
//     std::vector<std::string> ur_namespaces;
//     ur_namespaces.push_back("left");
//     ur_namespaces.push_back("right");
//     UrLogger ur_logger(nh, ur_namespaces);
//     ur_logger.start(100);

//     ros::waitForShutdown();
//     ur_logger.stop();

//     ros::shutdown();
//     return 0;
// }

