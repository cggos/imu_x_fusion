#pragma once

#include <iostream>

#include "common/state.hpp"

namespace cg {

constexpr int kMeasDim = 6;

enum JACOBIAN_MEASUREMENT { HX_X, NEGATIVE_RX_X };  // h(x)/delta X, -r(x)/delta X

class Odom6Dof {
 public:
  static const JACOBIAN_MEASUREMENT kJacobMeasurement_ = JACOBIAN_MEASUREMENT::NEGATIVE_RX_X;

  /**
   * @brief h(x)/delta X
   *
   * @param vo_q
   * @param T
   * @return
   */
  static Eigen::Matrix<double, kMeasDim, kStateDim> measurementH(const Eigen::Quaterniond &vo_q,
                                                                 const Eigen::Isometry3d &T,
                                                                 const Eigen::Isometry3d &Tvw,
                                                                 const Eigen::Isometry3d &Tcb) {
    Eigen::Matrix<double, kMeasDim, kStateDim> H;
    H.setZero();

    const Eigen::Matrix3d &Rvw = Tvw.linear();

    Eigen::Quaterniond q_vw(Rvw);
    Eigen::Quaterniond q_cb(Tcb.linear());
    Eigen::Quaterniond q(T.linear());

    switch (kJacobMeasurement_) {
      case JACOBIAN_MEASUREMENT::HX_X: {
        H.block<3, 3>(0, 0) = Rvw;
        if (State::kAngError == ANGULAR_ERROR::LOCAL_ANGULAR_ERROR) {
          H.block<3, 3>(0, 6) = -Rvw * T.linear() * skew_matrix(Tcb.inverse().translation());
          H.block<3, 3>(3, 6) =
              (quat_left_matrix((q_vw * q).normalized()) * quat_right_matrix(q_cb.conjugate())).block<3, 3>(1, 1);
        } else {
          H.block<3, 3>(0, 6) = -Rvw * skew_matrix(T.linear() * Tcb.inverse().translation());
          H.block<3, 3>(3, 6) =
              (quat_left_matrix(q_vw) * quat_right_matrix((q * q_cb.conjugate()).normalized())).block<3, 3>(1, 1);
        }
      } break;
      case JACOBIAN_MEASUREMENT::NEGATIVE_RX_X: {
        Eigen::Matrix4d m4;
        H.block<3, 3>(0, 0) = -Rvw;
        if (State::kAngError == ANGULAR_ERROR::LOCAL_ANGULAR_ERROR) {
          H.block<3, 3>(0, 6) = Rvw * T.linear() * skew_matrix(Tcb.inverse().translation());
          m4 = quat_left_matrix((vo_q.conjugate() * q_vw * q).normalized()) * quat_right_matrix(q_cb.conjugate());
          H.block<3, 3>(3, 6) = -m4.block<3, 3>(1, 1);
        } else {
          H.block<3, 3>(0, 6) = -Rvw * skew_matrix(T.linear() * Tcb.inverse().translation());
          m4 = quat_left_matrix(q_vw) * quat_right_matrix((q * (vo_q * q_cb).conjugate()).normalized());
          H.block<3, 3>(3, 6) = -m4.block<3, 3>(1, 1);
        }
        H *= -1.0;
      } break;
    }

    return H;
  }

  static void check_jacobian(const Eigen::Quaterniond &vo_q,
                             const Eigen::Isometry3d &Twb,
                             const Eigen::Isometry3d &Tvw,
                             const Eigen::Isometry3d &Tcb) {
    Eigen::Vector3d delta(0.0012, -0.00034, -0.00056);

    // perturbation on t
    Eigen::Isometry3d T1 = Twb;
    T1.translation() += delta;

    // perturbation on R
    Eigen::Isometry3d T2 = Twb;
    T2.linear() = State::rotation_update(T2.linear(), State::delta_rot_mat(delta, 1));

    Eigen::Isometry3d Tx0 = Tvw * Twb * Tcb.inverse();
    Eigen::Isometry3d Tx1 = Tvw * T1 * Tcb.inverse();
    Eigen::Isometry3d Tx2 = Tvw * T2 * Tcb.inverse();

    auto H = Odom6Dof::measurementH(vo_q, Twb, Tvw, Tcb);

    std::cout << "---------------------" << std::endl;
    std::cout << "(purt t) p res: " << (Tx1.translation() - Tx0.translation()).transpose() << std::endl;
    std::cout << "(purt t) p Hx: " << (H.block<3, 3>(0, 0) * delta).transpose() << std::endl;

    std::cout << "(purt R) p res: " << (Tx2.translation() - Tx0.translation()).transpose() << std::endl;
    std::cout << "(purt R) p Hx: " << (H.block<3, 3>(0, 6) * delta).transpose() << std::endl;

    std::cout << "(purt R) q res: " << State::rotation_residual(Tx2.linear(), Tx0.linear()).transpose() << std::endl;
    std::cout << "(purt R) q Hx: " << (H.block<3, 3>(3, 6) * delta).transpose() << std::endl;
    std::cout << "---------------------" << std::endl;
  }
};

}  // namespace cg
