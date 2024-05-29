#pragma once

#include "common/state.hpp"

namespace cg {

class StateEstimator {
 public:
  StateEstimator() { state_ptr_ = std::make_shared<State>(); }

 public:
  State::Ptr state_ptr_;
};

}  // namespace cg