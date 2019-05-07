//
// Created by Chunting on 27.12.16.
//

#ifndef PROJECT_TRANSFORM_H
#define PROJECT_TRANSFORM_H

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <geometry_msgs/TwistStamped.h>
// KDL
#include <kdl/frames_io.hpp>

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
namespace dual_arm_toolbox{
    class Transform {
    protected:
    public:
        static void transformPoseToKDL(geometry_msgs::Pose pose, KDL::Frame& kdl_frame);
        static void transformKDLtoPose(KDL::Frame kdl_frame, geometry_msgs::Pose& pose);
        static void transformVector6dtoTwist(Vector6d twist_vector, geometry_msgs::Twist &twist);
    };
}

#endif //PROJECT_TRANSFORM_H
