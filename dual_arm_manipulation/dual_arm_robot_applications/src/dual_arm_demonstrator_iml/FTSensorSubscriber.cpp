//
// Created by Chunting  on 20.10.16.
//
#include "dual_arm_demonstrator_iml/FTSensorSubscriber.h"

FTSensorSubscriber::FTSensorSubscriber(ros::NodeHandle &nh, std::string ur_namespace) : nh_(nh), ur_namespace_(ur_namespace)
{
    wrench_external_.setZero();
    wrench_filter_factor_ = 0.1;
    force_dead_zone_thres_ = 3;
    torque_dead_zone_thres_ = 0.5;
    sub_wrench_external_ = nh_.subscribe(ur_namespace_ + "/robotiq_ft_wrench", 1, &FTSensorSubscriber::wrenchCallback, this);
    // filtered_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>(ur_namespace_ + "/robotiq_ft_wrench", 1);
    ros::ServiceClient client = nh_.serviceClient<robotiq_ft_sensor::sensor_accessor>(ur_namespace_ + "/robotiq_ft_sensor_acc");
    robotiq_ft_sensor::sensor_accessor srv;

    if (ros::ok())
    {
        srv.request.command = "SET ZRO";
        if (client.call(srv))
        {
            ROS_INFO("---------------------ret: %s", srv.response.res.c_str());
        } else {
            ROS_ERROR("Failed SET ZRO");
        }
    }
}

void FTSensorSubscriber::wrenchCallback(const geometry_msgs::WrenchStamped::Ptr &msg)
{
    Vector6d wrench_ft_frame;
    // Reading the FT-sensor in its own frame (robotiq_ft_frame_id)
    wrench_ft_frame << msg->wrench.force.x, msg->wrench.force.y,
        msg->wrench.force.z, msg->wrench.torque.x,
        msg->wrench.torque.y, msg->wrench.torque.z;

    // Dead zone for the FT sensor
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
    // ROS_INFO_STREAM("After dead zone wrench_ft_frame: \n" << wrench_ft_frame);
    // Filter and update
    wrench_external_ = (1 - wrench_filter_factor_) * wrench_external_ +
                       wrench_filter_factor_ * wrench_ft_frame;
    last_wrench_msg_.header = msg->header;
    last_wrench_msg_.wrench.force.x = wrench_external_(0);
    last_wrench_msg_.wrench.force.y = wrench_external_(1);
    last_wrench_msg_.wrench.force.z = wrench_external_(2);
    last_wrench_msg_.wrench.torque.x = wrench_external_(3);
    last_wrench_msg_.wrench.torque.y = wrench_external_(4);
    last_wrench_msg_.wrench.torque.z = wrench_external_(5);
    // filtered_wrench_pub_.publish(last_wrench_msg_);
    // std::string topic = ur_namespace_ + "/robotiq_ft_wrench";
    // ROS_INFO("I publish last_wrench_msg_ to [%s]: Frame_id [%s] Time [%f] FX[%f] FY[%f] FZ[%f] MX[%f] MY[%f] MZ[%f]",
    //     topic.c_str(),
    //     last_wrench_msg_.header.frame_id.c_str(),   // robotiq_ft_frame_id
    //     last_wrench_msg_.header.stamp.toSec(),
    //     last_wrench_msg_.wrench.force.x, last_wrench_msg_.wrench.force.y, last_wrench_msg_.wrench.force.z,
    //     last_wrench_msg_.wrench.torque.x, last_wrench_msg_.wrench.torque.y, last_wrench_msg_.wrench.torque.z);
}
