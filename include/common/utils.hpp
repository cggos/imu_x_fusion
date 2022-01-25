#pragma once

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

namespace cg {

const double kDegreeToRadian = M_PI / 180.;

class Utils {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static Eigen::Isometry3d getTransformEigen(const ros::NodeHandle &nh, const std::string &field) {
    Eigen::Isometry3d T;
    cv::Mat c = getTransformCV(nh, field);

    T.linear()(0, 0) = c.at<double>(0, 0);
    T.linear()(0, 1) = c.at<double>(0, 1);
    T.linear()(0, 2) = c.at<double>(0, 2);
    T.linear()(1, 0) = c.at<double>(1, 0);
    T.linear()(1, 1) = c.at<double>(1, 1);
    T.linear()(1, 2) = c.at<double>(1, 2);
    T.linear()(2, 0) = c.at<double>(2, 0);
    T.linear()(2, 1) = c.at<double>(2, 1);
    T.linear()(2, 2) = c.at<double>(2, 2);
    T.translation()(0) = c.at<double>(0, 3);
    T.translation()(1) = c.at<double>(1, 3);
    T.translation()(2) = c.at<double>(2, 3);
    return T;
  }

  static cv::Mat getTransformCV(const ros::NodeHandle &nh, const std::string &field) {
    cv::Mat T;
    try {
      // first try reading kalibr format
      T = getKalibrStyleTransform(nh, field);
    } catch (std::runtime_error &e) {
      // maybe it's the old style format?
      ROS_DEBUG_STREAM("cannot read transform " << field << " in kalibr format, trying old one!");
      try {
        T = getVec16Transform(nh, field);
      } catch (std::runtime_error &e) {
        std::string msg = "cannot read transform " + field + " error: " + e.what();
        ROS_ERROR_STREAM(msg);
        throw std::runtime_error(msg);
      }
    }
    return T;
  }

  static cv::Mat getVec16Transform(const ros::NodeHandle &nh, const std::string &field) {
    std::vector<double> v;
    nh.getParam(field, v);
    if (v.size() != 16) {
      throw std::runtime_error("invalid vec16!");
    }
    cv::Mat T = cv::Mat(v).clone().reshape(1, 4);  // one channel 4 rows
    return T;
  }

  static cv::Mat getKalibrStyleTransform(const ros::NodeHandle &nh, const std::string &field) {
    cv::Mat T = cv::Mat::eye(4, 4, CV_64FC1);
    XmlRpc::XmlRpcValue lines;
    if (!nh.getParam(field, lines)) {
      throw(std::runtime_error("cannot find transform " + field));
    }
    if (lines.size() != 4 || lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      throw(std::runtime_error("invalid transform " + field));
    }
    for (int i = 0; i < lines.size(); i++) {
      if (lines.size() != 4 || lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        throw(std::runtime_error("bad line for transform " + field));
      }
      for (int j = 0; j < lines[i].size(); j++) {
        if (lines[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
          throw(std::runtime_error("bad value for transform " + field));
        } else {
          T.at<double>(i, j) = static_cast<double>(lines[i][j]);
        }
      }
    }
    return T;
  }

  static double condition_number(const Eigen::MatrixXd &A) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd singularValues;
    singularValues.resize(svd.singularValues().rows(), 1);
    singularValues = svd.singularValues();
    double cond = singularValues(0, 0) / singularValues(singularValues.rows() - 1, 0);
    return std::abs(cond);
  }

  static Eigen::Matrix3d skew_matrix(const Eigen::Vector3d &v) {
    Eigen::Matrix3d w;
    w << 0., -v(2), v(1), v(2), 0., -v(0), -v(1), v(0), 0.;
    return w;
  }

  static Eigen::Matrix4d quat_left_matrix(const Eigen::Quaterniond &q) {
    Eigen::Matrix4d m4 = Eigen::Matrix4d::Zero();
    m4.block<3, 1>(1, 0) = q.vec();
    m4.block<1, 3>(0, 1) = -q.vec();
    m4.block<3, 3>(1, 1) = skew_matrix(q.vec());
    m4 += Eigen::Matrix4d::Identity() * q.w();
    return m4;
  }

  static Eigen::Matrix4d quat_right_matrix(const Eigen::Quaterniond &q) {
    Eigen::Matrix4d m4 = Eigen::Matrix4d::Zero();
    m4.block<3, 1>(1, 0) = q.vec();
    m4.block<1, 3>(0, 1) = -q.vec();
    m4.block<3, 3>(1, 1) = -skew_matrix(q.vec());
    m4 += Eigen::Matrix4d::Identity() * q.w();
    return m4;
  }

  static Eigen::Vector3d rot_mat_to_vec(const Eigen::Matrix3d &R) {
    Eigen::Vector3d vec_r;
    cv::Mat Rmat, rvec;
    cv::eigen2cv(R, Rmat);
    cv::Rodrigues(Rmat, rvec);
    cv::cv2eigen(rvec, vec_r);
    return vec_r;
  }

  static Eigen::Matrix3d rot_vec_to_mat(const Eigen::Vector3d &rvec) {
    return Eigen::AngleAxisd(rvec.norm(), rvec.normalized()).toRotationMatrix();
  }
};

}  // namespace cg