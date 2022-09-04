#pragma once

#include <ceres/ceres.h>

#include "common/utils.hpp"
#include "estimator/map.hpp"
#include "sensor/odom_6dof.hpp"

namespace cg {

class PoseLocalParameterization : public ceres::LocalParameterization {
  template <typename Derived>
  inline static Eigen::Quaternion<typename Derived::Scalar> quat_from_delta(const Eigen::MatrixBase<Derived>& theta) {
    typedef typename Derived::Scalar Scalar_t;
    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0);
    dq.w() = static_cast<Scalar_t>(1.0);
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
  }

  virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const {
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = quat_from_delta(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
  }

  virtual bool ComputeJacobian(const double* x, double* jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();
    return true;
  }

  virtual int GlobalSize() const { return 7; };
  virtual int LocalSize() const { return 6; };
};

class MAPCostFunctor : public ceres::SizedCostFunction<6, 7> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MAPCostFunctor(const FactorPtr& factor_odom6dof_ptr,
                 const Eigen::Isometry3d& Tcb,
                 const Eigen::Isometry3d& Tvw,
                 const Eigen::Isometry3d& TvoB,
                 const Eigen::Matrix<double, kMeasDim, kMeasDim>& R)
      : factor_odom6dof_ptr_(factor_odom6dof_ptr) {
    Tcb_ = Tcb;
    Tvw_ = Tvw;
    Tvo_obs_ = TvoB;

    Eigen::Matrix<double, kMeasDim, kMeasDim> cov;
    // cov = R;
    cov.setIdentity();
    Lt_ = Eigen::LLT<Eigen::Matrix<double, kMeasDim, kMeasDim>>(cov).matrixL().transpose();
    Lt_.setIdentity();
  }

  virtual ~MAPCostFunctor() {}

  virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> vec_pq(parameters[0]);

    Eigen::Isometry3d Twb_i;  // x_i
    Twb_i.translation() = vec_pq.head(3);
    Twb_i.linear() = Eigen::Quaterniond(vec_pq.tail(4).data()).toRotationMatrix();

    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
    residual = factor_odom6dof_ptr_->measurement_residual(Twb_i.matrix(), Tvo_obs_.matrix());
    residual = Lt_ * residual;

    // J = r(x)/delta X
    const auto& J = -1.0 * factor_odom6dof_ptr_->measurement_jacobian(Twb_i.matrix(), Tvo_obs_.matrix());
    if (jacobians != nullptr) {
      if (jacobians[0] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> J0(jacobians[0]);

        Eigen::Matrix<double, 6, 6> Jpose;
        Jpose.leftCols(3) = J.block<6, 3>(0, 0);
        Jpose.rightCols(3) = J.block<6, 3>(0, 6);

        J0.leftCols(6) = Lt_ * Jpose;
        J0.rightCols(1).setZero();
      }
    }

    factor_odom6dof_ptr_->check_jacobian(Twb_i.matrix(), Tvo_obs_.matrix());  // for debug

    return true;
  }

 private:
  Eigen::Isometry3d Tcb_;
  Eigen::Isometry3d Tvw_;
  Eigen::Isometry3d Tvo_obs_;
  Eigen::Matrix<double, kMeasDim, kMeasDim> Lt_;

  FactorPtr factor_odom6dof_ptr_;
};

}  // namespace cg