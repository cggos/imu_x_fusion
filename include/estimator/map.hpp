#pragma once

#include "estimator/estimator.hpp"
#include "fusion/predictor.hpp"
#include "fusion/updator.hpp"

namespace cg {

class MAP : public StateEstimator, public Predictor, public Updator {
 public:
  MAP(double acc_n = 1e-2, double gyr_n = 1e-4, double acc_w = 1e-6, double gyr_w = 1e-8)
      : Predictor(state_ptr_, acc_n, gyr_n, acc_w, gyr_w) {}

  void predict(ImuDataConstPtr last_imu, ImuDataConstPtr curr_imu) {
    State last_state = *state_ptr_;

    state_ptr_->timestamp = curr_imu->timestamp;

    imu_model_.propagate_state(last_imu, curr_imu, last_state, *state_ptr_);
  }

  ~MAP() {}
};

using MAPPtr = std::shared_ptr<MAP>;

}  // namespace cg