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
          ur_listeners_.push_back(new UR_Message_Listener(nh_, ur_namespaces[i], logfolder_name_));
    }

    delimiter_ = ',';

    ROS_INFO("Initialising UR Logger"); //it needs to be waited until msgs can be received
    sleep(1);
}

UR_Logger::~UR_Logger()
{
    logfile_.close(); // close file when shutdown
    for (int i = 0; i < ur_listeners_.size(); i++)
    {
        delete ur_listeners_[i];
    }
}

void UR_Logger::start(int log_rate)
{
    if (logfile_name_ == "" || logfile_name_command_ == "")
    { //automatically generate a name if no name specified
        generate_logfile_name();
    }

    // write headline
    logfile_.open(logfile_name_.c_str(), std::ofstream::out | std::ofstream::trunc); //generate new file from beginning
    if (logfile_.is_open())
    {
        for (int i = 0; i < ur_listeners_.size(); i++)
        {
            logfile_ << headline(*ur_listeners_[i]);
            if (i < (ur_listeners_.size() - 1))
            {
                logfile_ << delimiter_;
            }
        }
        logfile_ << std::endl;
        ROS_INFO("Writing log at %iHz to %s. Press Ctrl-C to stop.", log_rate, logfile_name_.c_str());
    }else{
        ROS_ERROR("UrLogger failed! log file %s", logfile_name_.c_str());
    }

    logfile_command_.open(logfile_name_command_.c_str(), std::ofstream::out | std::ofstream::trunc); //generate new file from beginning
    if (logfile_command_.is_open())
    {
        logfile_command_ << "PC_time" << delimiter_ << "head_time";
        for (int i = 0; i < ur_listeners_.size(); i++)
        {
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
    log_duration = 1.0 / log_rate;
    timer_ = nh_.createTimer(ros::Duration().fromSec(log_duration), &UR_Logger::logCallback, this);
}

void UR_Logger::stop()
{
    ROS_INFO("Stopped logging");
    timer_.stop();
    logfile_.close();
    logfile_command_.close();
}

void UR_Logger::generate_logfile_name()
{ //automatically generate a name
    time_t rawtime;
    struct tm *timeinfo;
    char buffer[20];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, 20, "%Y_%m_%d_%H_%M%S", timeinfo);
    std::string log_suffix = buffer;
    logfile_name_ = logfolder_name_ + "ur_log.csv";
    logfile_name_command_ = logfolder_name_ + "ur_command.csv";
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
    std::string tool_pose_state_prefix = ur_listener.ur_namespace_ + "_tool_pose_state_";
    std::string tool_vel_state_prefix = ur_listener.ur_namespace_ + "_tool_vel_state_";
    std::string wrench_prefix = ur_listener.ur_namespace_;
    ss << delimiter_ << tool_pose_state_prefix << "x"
       << delimiter_ << tool_pose_state_prefix << "y"
       << delimiter_ << tool_pose_state_prefix << "z"
       << delimiter_ << tool_pose_state_prefix << "qx"
       << delimiter_ << tool_pose_state_prefix << "qy"
       << delimiter_ << tool_pose_state_prefix << "qz"
       << delimiter_ << tool_pose_state_prefix << "qw"
       << delimiter_ << tool_vel_state_prefix << "vx"
       << delimiter_ << tool_vel_state_prefix << "vy"
       << delimiter_ << tool_vel_state_prefix << "vz"
       << delimiter_ << tool_vel_state_prefix << "ax"
       << delimiter_ << tool_vel_state_prefix << "ay"
       << delimiter_ << tool_vel_state_prefix << "az"
       << delimiter_ << wrench_prefix << "_Fx"
       << delimiter_ << wrench_prefix << "_Fy"
       << delimiter_ << wrench_prefix << "_Fz"
       << delimiter_ << wrench_prefix << "_Fs"
       << delimiter_ << wrench_prefix << "x"
       << delimiter_ << wrench_prefix << "y"
       << delimiter_ << wrench_prefix << "z"
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
    converter << delimiter_ << ur_listener.last_m_tcp_pose_msg_.pose.position.x
              << delimiter_ << ur_listener.last_m_tcp_pose_msg_.pose.position.y
              << delimiter_ << ur_listener.last_m_tcp_pose_msg_.pose.position.z;
    // append Cartesian rotation quaternion
    converter << delimiter_ << ur_listener.last_m_tcp_pose_msg_.pose.orientation.x
              << delimiter_ << ur_listener.last_m_tcp_pose_msg_.pose.orientation.y
              << delimiter_ << ur_listener.last_m_tcp_pose_msg_.pose.orientation.z
              << delimiter_ << ur_listener.last_m_tcp_pose_msg_.pose.orientation.w;

    // append measured Cartesian speed
    converter << delimiter_ << ur_listener.last_m_tcp_vel_msg_.twist.linear.x
              << delimiter_ << ur_listener.last_m_tcp_vel_msg_.twist.linear.y
              << delimiter_ << ur_listener.last_m_tcp_vel_msg_.twist.linear.z
              << delimiter_ << ur_listener.last_m_tcp_vel_msg_.twist.angular.x
              << delimiter_ << ur_listener.last_m_tcp_vel_msg_.twist.angular.y
              << delimiter_ << ur_listener.last_m_tcp_vel_msg_.twist.angular.z;

    // // append tcp wrench force
    converter << delimiter_ << ur_listener.last_wrench_msg_.wrench.force.x
              << delimiter_ << ur_listener.last_wrench_msg_.wrench.force.y
              << delimiter_ << ur_listener.last_wrench_msg_.wrench.force.z
              << delimiter_<< sqrt(ur_listener.last_wrench_msg_.wrench.force.x * ur_listener.last_wrench_msg_.wrench.force.x + ur_listener.last_wrench_msg_.wrench.force.y * ur_listener.last_wrench_msg_.wrench.force.y + ur_listener.last_wrench_msg_.wrench.force.z * ur_listener.last_wrench_msg_.wrench.force.z);
    // tcp wrench torque
    converter << delimiter_ << ur_listener.last_wrench_msg_.wrench.torque.x
              << delimiter_ << ur_listener.last_wrench_msg_.wrench.torque.y
              << delimiter_ << ur_listener.last_wrench_msg_.wrench.torque.z;

    converter << delimiter_ << ur_listener.last_offset_msg_.point.x
              << delimiter_ << ur_listener.last_offset_msg_.point.y
              << delimiter_ << ur_listener.last_offset_msg_.point.z;

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
    // ss << "PC_time" << delimiter_ << "head_time";

    // append position command
    std::string command_pos_prefix = "c_joint_pos_" + ur_listener.ur_namespace_ + "_";
    for (int i = 0; i < ur_listener.last_joint_state_msg_.name.size(); i++)
    {
        ss << delimiter_ << command_pos_prefix << joint_names[i];
    }

    // append command velocity
    std::string command_vel_prefix = "c_joint_vel_" + ur_listener.ur_namespace_ + "_";
    for (int i = 0; i < ur_listener.last_joint_state_msg_.name.size(); i++)
    {
        ss << delimiter_ << command_vel_prefix << joint_names[i];
    }

    // std::cout << ss.str() << std::endl;

    return ss.str();
}
std::string UR_Logger::data_line_command(UR_Message_Listener &ur_listener)
{
    std::ostringstream converter; // stream used to convert numbers to string
    //  Only record the new data
    if (ur_listener.newTrajectory == true)
    {
        for (unsigned int i = 0; i < ur_listener.last_trajectory_msg_.joint_trajectory.points.size(); i++)
        {
            // append time      PCtime+time_from_start
            // It waits for 1 sec after pulish the trojectory
            double timeStamp = (stopwatch_.elapsed().toSec()) + ur_listener.last_trajectory_msg_.joint_trajectory.points[i].time_from_start.toSec() + 1;
            converter << timeStamp << delimiter_ << ur_listener.last_trajectory_msg_.joint_trajectory.points[i].time_from_start.toSec();
            int jn = ur_listener.last_trajectory_msg_.joint_trajectory.joint_names.size();
            // joint position of left arm
            if (jn == 12)
            {

                for (unsigned int a = 0; a < 6; a++)
                {
                    converter << delimiter_ << ur_listener.last_trajectory_msg_.joint_trajectory.points[i].positions[a];
                }
                // joint velocity of left arm
                for (unsigned int a = 0; a < 6; a++)
                {
                    converter << delimiter_ << ur_listener.last_trajectory_msg_.joint_trajectory.points[i].velocities[a];
                }
                // joint position of right arm
                for (unsigned int a = 6; a < ur_listener.last_trajectory_msg_.joint_trajectory.points[i].positions.size(); a++)
                {
                    converter << delimiter_ << ur_listener.last_trajectory_msg_.joint_trajectory.points[i].positions[a];
                }
                // joint velocity of right arm
                for (unsigned int a = 6; a < ur_listener.last_trajectory_msg_.joint_trajectory.points[i].positions.size(); a++)
                {
                    converter << delimiter_ << ur_listener.last_trajectory_msg_.joint_trajectory.points[i].velocities[a];
                }
            }
            else if (jn == 6)
            {
                std::string jointname = ur_listener.last_trajectory_msg_.joint_trajectory.joint_names[0];
                std::string right = "right_";
                if (jointname.compare(0, right.size(), right) == 0)
                {
                    converter << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_;
                    converter << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_;
                    for (unsigned int a = 0; a < 6; a++)
                    {
                        converter << delimiter_ << ur_listener.last_trajectory_msg_.joint_trajectory.points[i].positions[a];
                    }
                    // joint velocity of right arm
                    for (unsigned int a = 0; a < 6; a++)
                    {
                        converter << delimiter_ << ur_listener.last_trajectory_msg_.joint_trajectory.points[i].velocities[a];
                    }
                }
                else
                {
                    for (unsigned int a = 0; a < 6; a++)
                    {
                        converter << delimiter_ << ur_listener.last_trajectory_msg_.joint_trajectory.points[i].positions[a];
                    }
                    // joint velocity of left arm
                    for (unsigned int a = 0; a < 6; a++)
                    {
                        converter << delimiter_ << ur_listener.last_trajectory_msg_.joint_trajectory.points[i].velocities[a];
                    }
                    converter << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_;
                    converter << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_ << delimiter_;
                }
            }
            converter << std::endl;
        }
        ur_listener.last_trajectory_msg_.joint_trajectory.points.clear();
        ur_listener.newTrajectory = false;
    }

    return converter.str();
}

void UR_Logger::logCallback(const ros::TimerEvent &)
{
    for (int i = 0; i < ur_listeners_.size(); i++)
    {
        logfile_ << data_line(*ur_listeners_[i]);
        if (i < (ur_listeners_.size() - 1))
        {
            logfile_ << delimiter_;
        }
    }
    logfile_ << std::endl;

    //for(int i = 0; i < ur_listeners_.size(); i++){
    if (ur_listeners_[0]->newTrajectory)
        logfile_command_ << data_line_command(*ur_listeners_[0]);
    //}
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

