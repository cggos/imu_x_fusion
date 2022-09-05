#pragma once

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/edge_se3_prior.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include "sensor/odom_6dof.hpp"

namespace cg {

class VertexPose : public g2o::BaseVertex<6, Eigen::Isometry3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual bool read(std::istream& in) {}
  virtual bool write(std::ostream& out) const {}

  virtual void setToOriginImpl() { _estimate = Eigen::Isometry3d::Identity(); }

  virtual void oplusImpl(const double* update) {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> dpose(update);
    State::update_pose(_estimate, dpose);
  }
};

class EdgePose : public g2o::BaseUnaryEdge<6, Eigen::Isometry3d, VertexPose> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgePose(const FactorPtr& factor_odom6dof_ptr) : BaseUnaryEdge(), factor_odom6dof_ptr_(factor_odom6dof_ptr) {}

  virtual bool read(std::istream& in) {}

  virtual bool write(std::ostream& out) const {
    out << chi2();
    return out.good();
  }

  void computeError() {
    const VertexPose* v_pose = static_cast<const VertexPose*>(_vertices[0]);
    _error = factor_odom6dof_ptr_->measurement_residual(v_pose->estimate().matrix(), _measurement.matrix());
  }

  virtual void linearizeOplus() {
    const VertexPose* v_pose = static_cast<const VertexPose*>(_vertices[0]);
    auto J = -1.0 * factor_odom6dof_ptr_->measurement_jacobian(v_pose->estimate().matrix(), _measurement.matrix());
    _jacobianOplusXi.leftCols(3) = J.leftCols(3);
    _jacobianOplusXi.rightCols(3) = J.block<6, 3>(0, 6);
  }

  Eigen::Matrix<double, 6, 6> GetHessian() {
    linearizeOplus();
    return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
  }

  void gating_test() {
    computeError();

    if (chi2() > th_huber_)
      setLevel(1);
    else
      setLevel(0);
  }

 public:
  const float th_huber_ = 0.8 * std::sqrt(12.592);  // chi-square P for 6DoF

 private:
  FactorPtr factor_odom6dof_ptr_;
};

}  // namespace cg