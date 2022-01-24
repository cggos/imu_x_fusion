#pragma once

#include "common/utils.h"
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

  ~EKF() {}
};

using EKFPtr = std::unique_ptr<EKF>;

}  // namespace cg