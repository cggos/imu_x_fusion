#pragma once

#include "fusion/observer.hpp"

namespace cg {

class Updator {
 public:
  Updator() {}

  Updator(const Updator &) = delete;

 public:
  ObserverPtr observer_ptr_;
};

}  // namespace cg