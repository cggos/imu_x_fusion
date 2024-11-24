#pragma once

#include <Eigen/Core>

namespace cg {

class Factor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<Factor>;

  Factor() = default;

  Factor(const Factor &) = delete;

  virtual Eigen::MatrixXd measurement_residual(const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z) = 0;

  virtual Eigen::MatrixXd measurement_jacobian(const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z) = 0;

  virtual void check_jacobian(const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z) = 0;

  virtual ~Factor() {}
};

}  // namespace cg
