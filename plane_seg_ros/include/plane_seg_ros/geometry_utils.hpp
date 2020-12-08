#pragma once 
#include <string>
#include <Eigen/Dense>

namespace planeseg{

std::string vecToStr(const Eigen::Vector3f& iVec);
std::string rotToStr(const Eigen::Matrix3f& iRot);
void quat_to_euler(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);
Eigen::Vector3f convertRobotPoseToSensorLookDir(Eigen::Isometry3d robot_pose);

}
