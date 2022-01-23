#pragma once

#include "common/state.hpp"
#include "sensor/imu.hpp"

namespace cg {

class MAP {
 public:
  StatePtr state_ptr_;

  MAP() { state_ptr_ = std::make_shared<State>(); }

  void predict(ImuDataConstPtr last_imu, ImuDataConstPtr curr_imu) {
    State last_state = *state_ptr_;

    state_ptr_->timestamp = curr_imu->timestamp;

    imu_model_.propagate_state(last_imu, curr_imu, last_state, *state_ptr_);
  }

  ~MAP() {}

 public:
  IMU imu_model_;
};

using MAPPtr = std::shared_ptr<MAP>;

}  // namespace cg