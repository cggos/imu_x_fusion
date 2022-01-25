#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include "common/utils.hpp"

namespace cg {

constexpr double kG = 9.81007;

constexpr int kStateDim = 15;
constexpr int kNoiseDim = 12;

/**
 * @brief local or global angular error or rotation perturbation, ref: JoanSola ESKF 7.
 *
 * @details residual, jacobi and state update on rotation with the same methods
 *
 * @ref JoanSola ESKF 7.
 *
 */
enum ANGULAR_ERROR { LOCAL_ANGULAR_ERROR, GLOBAL_ANGULAR_ERROR };

using MatrixSD = Eigen::Matrix<double, kStateDim, kStateDim>;

class State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // error-state
  MatrixSD cov;

  // nominal-state
  Eigen::Vector3d p_wb_;
  Eigen::Vector3d v_wb_;
  Eigen::Matrix3d Rwb_;
  Eigen::Vector3d acc_bias;
  Eigen::Vector3d gyr_bias;

  double timestamp;

  static ANGULAR_ERROR kAngError;

  State() {
    cov.setZero();

    p_wb_.setZero();
    v_wb_.setZero();
    Rwb_.setIdentity();
    acc_bias.setZero();
    gyr_bias.setZero();
  }

  void set_bias(const Eigen::Vector3d &ba, const Eigen::Vector3d &bg) {
    acc_bias = ba;
    gyr_bias = bg;
  }

  const Eigen::Isometry3d pose() const {
    Eigen::Isometry3d Twb;
    Twb.linear() = Rwb_;
    Twb.translation() = p_wb_;
    return Twb;
  }

  const Eigen::Matrix<double, kStateDim, 1> vec() const {
    Eigen::Matrix<double, kStateDim, 1> vec;

    vec.segment<3>(0) = p_wb_;
    vec.segment<3>(3) = v_wb_;
    vec.segment<3>(6) = Utils::rot_mat_to_vec(Rwb_);
    vec.segment<3>(9) = acc_bias;
    vec.segment<3>(12) = gyr_bias;

    return vec;
  }

  void from_vec(const Eigen::Matrix<double, kStateDim, 1> &vec) {
    p_wb_ = vec.segment<3>(0);
    v_wb_ = vec.segment<3>(3);
    Rwb_ = Utils::rot_vec_to_mat(vec.segment<3>(6));
    acc_bias = vec.segment<3>(9);
    gyr_bias = vec.segment<3>(12);
  }

  /**
   * @brief Set the cov object
   *
   * @param sigma_p pos std, m
   * @param sigma_v vel std, m/s
   * @param sigma_rp roll pitch std
   * @param sigma_yaw yaw std
   * @param sigma_ba Acc bias
   * @param sigma_bg Gyr bias
   */
  void set_cov(double sigma_p, double sigma_v, double sigma_rp, double sigma_yaw, double sigma_ba, double sigma_bg) {
    cov.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * sigma_p * sigma_p;
    cov.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * sigma_v * sigma_v;
    cov.block<2, 2>(6, 6) = Eigen::Matrix2d::Identity() * sigma_rp * sigma_rp;
    cov(8, 8) = sigma_yaw * sigma_yaw;
    cov.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * sigma_ba * sigma_ba;
    cov.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * sigma_bg * sigma_bg;
  }

  State &operator=(const State &rhs) {
    if (this == &rhs) return *this;
    p_wb_ = rhs.p_wb_;
    v_wb_ = rhs.v_wb_;
    Rwb_ = rhs.Rwb_;
    acc_bias = rhs.acc_bias;
    gyr_bias = rhs.gyr_bias;
    return *this;
  }

  State operator+(const Eigen::Matrix<double, kStateDim, 1> &delta_x) const {
    State state;
    state.p_wb_ = this->p_wb_ + delta_x.block<3, 1>(0, 0);
    state.v_wb_ = this->v_wb_ + delta_x.block<3, 1>(3, 0);
    state.Rwb_ = rotation_update(this->Rwb_, delta_rot_mat(delta_x.block<3, 1>(6, 0)));
    state.acc_bias = this->acc_bias + delta_x.block<3, 1>(9, 0);
    state.gyr_bias = this->gyr_bias + delta_x.block<3, 1>(12, 0);
    return state;
  }

  Eigen::Matrix<double, kStateDim, 1> operator-(const State &rhs) const {
    Eigen::Matrix<double, kStateDim, 1> delta_x;
    delta_x.block<3, 1>(0, 0) = this->p_wb_ - rhs.p_wb_;
    delta_x.block<3, 1>(3, 0) = this->v_wb_ - rhs.v_wb_;
    delta_x.block<3, 1>(6, 0) = rotation_residual(this->Rwb_, rhs.Rwb_);
    delta_x.block<3, 1>(9, 0) = this->acc_bias - rhs.acc_bias;
    delta_x.block<3, 1>(12, 0) = this->gyr_bias - rhs.gyr_bias;
    return delta_x;
  }

  /**
   * @brief
   *
   * @param delta_rot_vec small rotation vector
   * @param flag 0: angle axis, 1: quaternion
   * @return Eigen::Matrix3d
   */
  static Eigen::Matrix3d delta_rot_mat(const Eigen::Vector3d &delta_rot_vec, int flag = 0) {
    Eigen::Matrix3d deltaR = Eigen::Matrix3d::Identity();
    if (flag == 0 && delta_rot_vec.norm() > DBL_EPSILON) {
      deltaR = Utils::rot_vec_to_mat(delta_rot_vec);
    }
    if (flag == 1) {
      Eigen::Quaterniond delta_q;
      delta_q.w() = 1;
      delta_q.vec() = 0.5 * delta_rot_vec;
      deltaR = delta_q.toRotationMatrix();
    }
    return deltaR;
  }

  /**
   * @brief Rwb + delta_rot_mat
   *
   * @details Rwb: 1) active 2) local-to-global
   *
   * @param Rwb
   * @param delta_rot_mat small rotation matrix, local or global perturbation
   * @return Eigen::Matrix3d
   */
  static Eigen::Matrix3d rotation_update(const Eigen::Matrix3d &Rwb, const Eigen::Matrix3d &delta_rot_mat) {
    Eigen::Matrix3d updatedR = Eigen::Matrix3d::Identity();
    switch (kAngError) {
      case ANGULAR_ERROR::LOCAL_ANGULAR_ERROR:
        updatedR = Rwb * delta_rot_mat;
        break;
      case ANGULAR_ERROR::GLOBAL_ANGULAR_ERROR:
        updatedR = delta_rot_mat * Rwb;
        break;
    }
    return updatedR;
  }

  /**
   * @brief Robs - Rest
   *
   * @details Robs, Rest: 1) active 2) local-to-global
   *
   * @param Robs
   * @param Rest
   * @return Eigen::Vector3d
   */
  static Eigen::Vector3d rotation_residual(const Eigen::Matrix3d &Robs, const Eigen::Matrix3d &Rest) {
    Eigen::Quaterniond q_res;
    switch (kAngError) {
      case ANGULAR_ERROR::LOCAL_ANGULAR_ERROR:
        q_res = Eigen::Quaterniond(Rest.transpose() * Robs);
        break;
      case ANGULAR_ERROR::GLOBAL_ANGULAR_ERROR:
        q_res = Eigen::Quaterniond(Robs * Rest.transpose());
        break;
    }
    return 2.0 * q_res.vec() / q_res.w();
  }
};
using StatePtr = std::shared_ptr<State>;

}  // namespace cg