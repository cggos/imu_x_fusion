#pragma once

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

namespace cg {

inline Eigen::Matrix3d skew_matrix(const Eigen::Vector3d &v) {
    Eigen::Matrix3d w;
    w << 0., -v(2), v(1),
        v(2), 0., -v(0),
        -v(1), v(0), 0.;

    return w;
}

Eigen::Isometry3d getTransformEigen(const ros::NodeHandle &nh, const std::string &field);

cv::Mat getTransformCV(const ros::NodeHandle &nh, const std::string &field);

cv::Mat getVec16Transform(const ros::NodeHandle &nh, const std::string &field);

cv::Mat getKalibrStyleTransform(const ros::NodeHandle &nh, const std::string &field);

Eigen::Matrix4d quat_left_matrix(const Eigen::Quaterniond &q);
Eigen::Matrix4d quat_right_matrix(const Eigen::Quaterniond &q);

}  // namespace cg
