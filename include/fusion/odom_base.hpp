#pragma once

#include "common/state.hpp"

namespace cg {
class OdomBase {
 public:
  OdomBase() { state_ptr_ = std::make_shared<State>(); }

 public:
  StatePtr state_ptr_;
};
}  // namespace cg