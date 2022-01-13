#pragma once

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

namespace cg {

Eigen::Isometry3d getTransformEigen(const ros::NodeHandle &nh, const std::string &field);

cv::Mat getTransformCV(const ros::NodeHandle &nh, const std::string &field);

cv::Mat getVec16Transform(const ros::NodeHandle &nh, const std::string &field);

cv::Mat getKalibrStyleTransform(const ros::NodeHandle &nh, const std::string &field);

Eigen::Matrix3d skew_matrix(const Eigen::Vector3d &v);

Eigen::Matrix4d quat_left_matrix(const Eigen::Quaterniond &q);
Eigen::Matrix4d quat_right_matrix(const Eigen::Quaterniond &q);

Eigen::Vector3d rot_mat_to_vec(const Eigen::Matrix3d &R);

Eigen::Matrix3d rot_vec_to_mat(const Eigen::Vector3d &rvec);

}  // namespace cg
