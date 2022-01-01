#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

namespace cg {

constexpr int kStateDim = 15;
constexpr int kNoiseDim = 12;
constexpr double kG = 9.81007;

const double kDegreeToRadian = M_PI / 180.;

using MatrixSD = Eigen::Matrix<double, kStateDim, kStateDim>;

/**
 * @brief local or global angular error or rotation perturbation, ref: JoanSola ESKF 7.
 *
 * @details residual, jacobi and state update on rotation with the same methods
 *
 * @ref JoanSola ESKF 7.
 *
 */
enum ANGULAR_ERROR { LOCAL_ANGULAR_ERROR, GLOBAL_ANGULAR_ERROR };

enum JACOBIAN_MEASUREMENT { HX_X, NEGATIVE_RX_X };  // h(x)/delta X, -r(x)/delta X

struct ImuData {
  double timestamp;
  Eigen::Vector3d acc;
  Eigen::Vector3d gyr;
};
using ImuDataPtr = std::shared_ptr<ImuData>;
using ImuDataConstPtr = std::shared_ptr<const ImuData>;

struct GpsData {
  double timestamp;

  Eigen::Vector3d lla;  // Latitude in degree, longitude in degree, and altitude in meter
  Eigen::Matrix3d cov;  // Covariance in m^2
};
using GpsDataPtr = std::shared_ptr<GpsData>;
using GpsDataConstPtr = std::shared_ptr<const GpsData>;

class State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // error-state
  MatrixSD cov;

  // nominal-state
  Eigen::Vector3d p_GI;
  Eigen::Vector3d v_GI;
  Eigen::Matrix3d r_GI;
  Eigen::Vector3d acc_bias;
  Eigen::Vector3d gyr_bias;

  double timestamp;

  static ANGULAR_ERROR kAngError;

  State() {
    cov.setZero();

    p_GI.setZero();
    v_GI.setZero();
    r_GI.setIdentity();
    acc_bias.setZero();
    gyr_bias.setZero();
  }

  const Eigen::Isometry3d pose() const {
    Eigen::Isometry3d Twb;
    Twb.linear() = r_GI;
    Twb.translation() = p_GI;
    return Twb;
  }

  State &operator=(const State &rhs) {
    if (this == &rhs) return *this;
    p_GI = rhs.p_GI;
    v_GI = rhs.v_GI;
    r_GI = rhs.r_GI;
    acc_bias = rhs.acc_bias;
    gyr_bias = rhs.gyr_bias;
    return *this;
  }

  State operator+(const Eigen::Matrix<double, kStateDim, 1> &delta_x) const {
    State state;
    state.p_GI = this->p_GI + delta_x.block<3, 1>(0, 0);
    state.v_GI = this->v_GI + delta_x.block<3, 1>(3, 0);
    state.r_GI = rotation_update(this->r_GI, delta_rot_mat(delta_x.block<3, 1>(6, 0)));
    state.acc_bias = this->acc_bias + delta_x.block<3, 1>(9, 0);
    state.gyr_bias = this->gyr_bias + delta_x.block<3, 1>(12, 0);
    return state;
  }

  Eigen::Matrix<double, kStateDim, 1> operator-(const State &rhs) const {
    Eigen::Matrix<double, kStateDim, 1> delta_x;
    delta_x.block<3, 1>(0, 0) = this->p_GI - rhs.p_GI;
    delta_x.block<3, 1>(3, 0) = this->v_GI - rhs.v_GI;
    delta_x.block<3, 1>(6, 0) = rotation_residual(this->r_GI, rhs.r_GI);
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
      deltaR = Eigen::AngleAxisd(delta_rot_vec.norm(), delta_rot_vec.normalized()).toRotationMatrix();
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