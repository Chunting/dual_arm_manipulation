//
// Created by Chunting  on 17.10.16.
//

#include "dual_arm_toolbox/Transform.h"

using namespace dual_arm_toolbox;
/*
geometry_msgs/Pose.msg:
    geometry_msgs/Point position [x y z]
    geometry_msgs/Quaternion orientation [x y z w]
*/
void Transform::transformPoseToKDL(geometry_msgs::Pose pose, KDL::Frame &kdl_frame)
{
    kdl_frame.M = kdl_frame.M.Quaternion(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    kdl_frame.p.x(pose.position.x);
    kdl_frame.p.y(pose.position.y);
    kdl_frame.p.z(pose.position.z);
}

void Transform::transformKDLtoPose(KDL::Frame kdl_frame, geometry_msgs::Pose &pose)
{
    kdl_frame.M.GetQuaternion(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    pose.position.x = kdl_frame.p.x();
    pose.position.y = kdl_frame.p.y();
    pose.position.z = kdl_frame.p.z();
}

void Transform::transformVector6dtoTwist(Vector6d twist_vector, geometry_msgs::Twist &twist)
{
  twist.linear.x = twist_vector(0);
  twist.linear.y = twist_vector(1);
  twist.linear.z = twist_vector(2);
  twist.angular.x = twist_vector(3);
  twist.angular.y = twist_vector(4);
  twist.angular.z = twist_vector(5);
}