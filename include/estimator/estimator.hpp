#pragma once

#include "common/state.hpp"

namespace cg {

class StateEstimator {
 public:
  StateEstimator() { state_ptr_ = std::make_shared<State>(); }

 public:
  StatePtr state_ptr_;
};

}  // namespace cg