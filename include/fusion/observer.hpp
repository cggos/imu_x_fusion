#pragma once

#include <Eigen/Core>

#include "fusion/factor.hpp"

namespace cg {

enum JACOBIAN_MEASUREMENT { HX_X, NEGATIVE_RX_X };  // h(x)/delta X, -r(x)/delta X

class Observer : public Factor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<Observer>;

  Observer() = default;

  Observer(const Observer &) = delete;

  virtual ~Observer() {}

  virtual Eigen::MatrixXd measurement_function(const Eigen::MatrixXd &mat_x) = 0;
};

}  // namespace cg