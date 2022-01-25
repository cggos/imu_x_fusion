#pragma once

#include "common/utils.hpp"
#include "estimator/kf.hpp"

namespace cg {

class EKF : public KF {
 public:
  StatePtr state_ptr_i_;  // for IEKF

  EKF() = delete;

  EKF(const EKF &) = delete;

  explicit EKF(double acc_n = 1e-2, double gyr_n = 1e-4, double acc_w = 1e-6, double gyr_w = 1e-8)
      : KF(acc_n, gyr_n, acc_w, gyr_w) {
    state_ptr_i_ = std::make_shared<State>();
  }

  /**
   * @brief predict procedure
   * @param last_imu
   * @param curr_imu
   */
  void predict(ImuDataConstPtr last_imu, ImuDataConstPtr curr_imu) {
    State last_state = *state_ptr_;

    state_ptr_->timestamp = curr_imu->timestamp;

    imu_model_.propagate_state(last_imu, curr_imu, last_state, *state_ptr_);
    imu_model_.propagate_state_cov(last_imu, curr_imu, last_state, *state_ptr_);
  }

  template <class H_type, class R_type, class K_type>
  void update_K(const Eigen::MatrixBase<H_type> &H, const Eigen::MatrixBase<R_type> &R, Eigen::MatrixBase<K_type> &K) {
    const auto &P = state_ptr_->cov;
    const R_type &S = H * P * H.transpose() + R;
    K = P * H.transpose() * S.inverse();
    // const Eigen::Matrix<double, kStateDim, R_type::RowsAtCompileTime> K = P * H.transpose() * S.inverse();
  }

  template <class H_type, class R_type, class K_type>
  void update_P(const Eigen::MatrixBase<H_type> &H,
                const Eigen::MatrixBase<R_type> &R,
                const Eigen::MatrixBase<K_type> &K) {
    const MatrixSD &I_KH = MatrixSD::Identity() - K * H;
    state_ptr_->cov = I_KH * state_ptr_->cov * I_KH.transpose() + K * R * K.transpose();
  }

  ~EKF() {}
};

using EKFPtr = std::unique_ptr<EKF>;

}  // namespace cg