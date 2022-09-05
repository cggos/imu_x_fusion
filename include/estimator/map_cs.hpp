#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "common/utils.hpp"
#include "estimator/map.hpp"
#include "sensor/odom_6dof.hpp"

namespace cg {

class QuatLocalParameterization : public ceres::LocalParameterization {
  template <typename Derived>
  inline static Eigen::Quaternion<typename Derived::Scalar> quat_from_delta(const Eigen::MatrixBase<Derived>& theta) {
    typedef typename Derived::Scalar Scalar_t;
    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0);

    dq.vec() = half_theta;
    // dq.w() = static_cast<Scalar_t>(1.0);

    double dq_square_norm = half_theta.squaredNorm();
    if (dq_square_norm <= 1)
      dq.w() = static_cast<Scalar_t>(std::sqrt(1 - dq_square_norm));
    else {
      dq.w() = static_cast<Scalar_t>(1.0);
      dq.normalize();
    }

    return dq;
  }

  virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const {
    Eigen::Map<const Eigen::Quaterniond> _q(x);
    Eigen::Quaterniond dq = quat_from_delta(Eigen::Map<const Eigen::Vector3d>(delta));
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta);
    q = (_q * dq).normalized();
    return true;
  }

  virtual bool ComputeJacobian(const double* x, double* jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
    j.topRows<3>().setIdentity();
    j.bottomRows<1>().setZero();
    return true;
  }

  virtual int GlobalSize() const { return 4; };
  virtual int LocalSize() const { return 3; };
};

class MAPCostFunctor : public ceres::SizedCostFunction<6, 3, 4> {
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
    Eigen::Map<const Eigen::Matrix<double, 3, 1>> vec_p(parameters[0]);
    Eigen::Map<const Eigen::Matrix<double, 4, 1>> vec_q(parameters[1]);

    Eigen::Isometry3d Twb_i;  // x_i
    Twb_i.translation() = vec_p;
    Twb_i.linear() = Eigen::Quaterniond(vec_q.data()).toRotationMatrix();

    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
    residual = factor_odom6dof_ptr_->measurement_residual(Twb_i.matrix(), Tvo_obs_.matrix());
    residual = Lt_ * residual;

    // J = r(x)/delta X
    const auto& J = -1.0 * factor_odom6dof_ptr_->measurement_jacobian(Twb_i.matrix(), Tvo_obs_.matrix());
    if (jacobians != nullptr) {
      if (jacobians[0] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> J0(jacobians[0]);
        J0 = Lt_ * J.block<6, 3>(0, 0);
      }
      if (jacobians[1] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> J1(jacobians[1]);
        J1.leftCols(3) = Lt_ * J.block<6, 3>(0, 6);
        J1.rightCols(1).setZero();
      }
    }

    // factor_odom6dof_ptr_->check_jacobian(Twb_i.matrix(), Tvo_obs_.matrix());  // for debug

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