#pragma once

#include <iostream>

#include "common/state.hpp"
#include "fusion/observer.hpp"

namespace cg {

constexpr int kMeasDim = 6;

class Odom6Dof : public Observer {
 public:
  Odom6Dof() = default;

  virtual ~Odom6Dof() {}

  void set_params(const Eigen::Isometry3d &Tvw, const Eigen::Isometry3d &Tcb) {
    Tvw_ = Tvw;
    Tcb_ = Tcb;
  }

  /**
   * @brief measurement estimation h(x), Twb in frame V --> Tc0cn
   *
   * @param mat_x
   * @return Eigen::MatrixXd
   */
  virtual Eigen::MatrixXd measurement_function(const Eigen::MatrixXd &mat_x) {
    Eigen::Isometry3d iso_x;
    iso_x.matrix() = mat_x;
    Eigen::Isometry3d Twb_in_V = Tvw_ * iso_x * Tcb_.inverse();
    return Twb_in_V.matrix();
  }

  /**
   * @brief residual = z - h(x)
   *
   * @param mat_x
   * @param mat_z
   * @return Eigen::MatrixXd
   */
  virtual Eigen::MatrixXd measurement_residual(const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z) {
    Eigen::Isometry3d ios_x_in_z;
    ios_x_in_z.matrix() = measurement_function(mat_x);

    Eigen::Isometry3d ios_z;
    ios_z.matrix() = mat_z;

    Eigen::Matrix<double, kMeasDim, 1> residual;
    residual.topRows(3) = ios_z.translation() - ios_x_in_z.translation();
    residual.bottomRows(3) = State::rotation_residual(ios_z.linear(), ios_x_in_z.linear());

    return residual;
  }

  /**
   * @brief h(x)/delta X or -r(x)/delta X
   *
   * @param mat_x
   * @param mat_z
   * @return Eigen::MatrixXd
   */
  virtual Eigen::MatrixXd measurement_jacobian(const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z) {
    Eigen::Matrix<double, kMeasDim, kStateDim> H;
    H.setZero();

    Eigen::Isometry3d T;
    T.matrix() = mat_x;

    Eigen::Isometry3d Tvo;
    Tvo.matrix() = mat_z;

    Eigen::Quaterniond vo_q(Tvo.linear());

    const Eigen::Matrix3d &Rvw = Tvw_.linear();

    Eigen::Quaterniond q_vw(Rvw);
    Eigen::Quaterniond q_cb(Tcb_.linear());
    Eigen::Quaterniond q(T.linear());

    // clang-format off
    switch (kJacobMeasurement_) {
      case JACOBIAN_MEASUREMENT::HX_X: {
        H.block<3, 3>(0, 0) = Rvw;
        if (State::kAngError == ANGULAR_ERROR::LOCAL_ANGULAR_ERROR) {
          H.block<3, 3>(0, 6) = -Rvw * T.linear() * Utils::skew_matrix(Tcb_.inverse().translation());
          H.block<3, 3>(3, 6) = (Utils::quat_left_matrix((q_vw * q).normalized()) * Utils::quat_right_matrix(q_cb.conjugate())).block<3, 3>(1, 1);
        } else {
          H.block<3, 3>(0, 6) = -Rvw * Utils::skew_matrix(T.linear() * Tcb_.inverse().translation());
          H.block<3, 3>(3, 6) = (Utils::quat_left_matrix(q_vw) * Utils::quat_right_matrix((q * q_cb.conjugate()).normalized())).block<3, 3>(1, 1);
        }
      } break;
      case JACOBIAN_MEASUREMENT::NEGATIVE_RX_X: {
        Eigen::Matrix4d m4;
        H.block<3, 3>(0, 0) = -Rvw;
        if (State::kAngError == ANGULAR_ERROR::LOCAL_ANGULAR_ERROR) {
          H.block<3, 3>(0, 6) = Rvw * T.linear() * Utils::skew_matrix(Tcb_.inverse().translation());
          m4 = Utils::quat_left_matrix((vo_q.conjugate() * q_vw * q).normalized()) * Utils::quat_right_matrix(q_cb.conjugate());
          H.block<3, 3>(3, 6) = -m4.block<3, 3>(1, 1);
        } else {
          H.block<3, 3>(0, 6) = -Rvw * Utils::skew_matrix(T.linear() * Tcb_.inverse().translation());
          m4 = Utils::quat_left_matrix(q_vw) * Utils::quat_right_matrix((q * (vo_q * q_cb).conjugate()).normalized());
          H.block<3, 3>(3, 6) = -m4.block<3, 3>(1, 1);
        }
        H *= -1.0;
      } break;
    }
    //clang-format on

    return H;
  }

  /**
   * @brief check jacobian
   * 
   * @param mat_x 
   * @param mat_z 
   */
  virtual void check_jacobian(const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z) {
    Eigen::Vector3d delta(0.0012, -0.00034, -0.00056);

    Eigen::Isometry3d T0;
    T0.matrix() = mat_x;
    
    // perturbation on t
    Eigen::Isometry3d T1 = T0;
    T1.translation() += delta;

    // perturbation on R
    Eigen::Isometry3d T2 = T0;
    T2.linear() = State::rotation_update(T2.linear(), State::delta_rot_mat(delta, 1));

    Eigen::Isometry3d Tx0 = Tvw_ * T0 * Tcb_.inverse();
    Eigen::Isometry3d Tx1 = Tvw_ * T1 * Tcb_.inverse();
    Eigen::Isometry3d Tx2 = Tvw_ * T2 * Tcb_.inverse();

    const auto &H = measurement_jacobian(mat_x, mat_z);

    std::cout << "---------------------" << std::endl;
    std::cout << "(purt t) p res: " << (Tx1.translation() - Tx0.translation()).transpose() << std::endl;
    std::cout << "(purt t) p Hx: " << (H.block<3, 3>(0, 0) * delta).transpose() << std::endl;

    std::cout << "(purt R) p res: " << (Tx2.translation() - Tx0.translation()).transpose() << std::endl;
    std::cout << "(purt R) p Hx: " << (H.block<3, 3>(0, 6) * delta).transpose() << std::endl;

    std::cout << "(purt R) q res: " << State::rotation_residual(Tx2.linear(), Tx0.linear()).transpose() << std::endl;
    std::cout << "(purt R) q Hx: " << (H.block<3, 3>(3, 6) * delta).transpose() << std::endl;
    std::cout << "---------------------" << std::endl;
  }

 private:
  static const JACOBIAN_MEASUREMENT kJacobMeasurement_ = JACOBIAN_MEASUREMENT::NEGATIVE_RX_X;
  Eigen::Isometry3d Tvw_;
  Eigen::Isometry3d Tcb_;
};
using Odom6DofPtr = std::shared_ptr<Odom6Dof>;

}  // namespace cg
