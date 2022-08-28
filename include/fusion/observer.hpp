#pragma once

#include <Eigen/Core>

#include "fusion/factor.hpp"

namespace cg {

enum JACOBIAN_MEASUREMENT { HX_X, NEGATIVE_RX_X };  // h(x)/delta X, -r(x)/delta X

class Observer : public Factor {
 public:
  Observer() = default;

  Observer(const Observer &) = delete;

  virtual ~Observer() {}

  virtual Eigen::MatrixXd measurement_function(const Eigen::MatrixXd &mat_x) = 0;
};
using ObserverPtr = std::shared_ptr<Observer>;

}  // namespace cg