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

class MAPCostFunctor : public ceres::SizedCostFunction<6, 3, 3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MAPCostFunctor(const MAPPtr& map_ptr,
                 const Eigen::Isometry3d& Tcb,
                 const Eigen::Isometry3d& Tvw,
                 const Eigen::Isometry3d& TvoB,
                 const Eigen::Matrix<double, kMeasDim, kMeasDim>& R)
      : map_ptr_(map_ptr) {
    Tcb_ = Tcb;
    Tvw_ = Tvw;
    Tvo_obs_ = TvoB;

    Eigen::Matrix<double, kMeasDim, kMeasDim> cov;
    // cov = R;
    cov.setIdentity();
    Lt_ = Eigen::LLT<Eigen::Matrix<double, kMeasDim, kMeasDim>>(cov).matrixL().transpose();
  }

  virtual ~MAPCostFunctor() {}

  virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    Eigen::Map<const Eigen::Matrix<double, 3, 1>> vec_p(parameters[0]);
    Eigen::Map<const Eigen::Matrix<double, 3, 1>> vec_R(parameters[1]);

    Eigen::Isometry3d Twb_i;  // x_i
    Twb_i.translation() = vec_p;
    Twb_i.linear() = Utils::rot_vec_to_mat(vec_R);

    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
    residual = map_ptr_->observer_ptr_->measurement_residual(Twb_i.matrix(), Tvo_obs_.matrix());
    residual = Lt_ * residual;

    const auto& J = -1.0 * map_ptr_->observer_ptr_->measurement_jacobian(Twb_i.matrix(), Tvo_obs_.matrix());
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> J0(jacobians[0]);
        J0 = Lt_ * J.block<6, 3>(0, 0);
      }
      if (jacobians[1] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> J1(jacobians[1]);
        J1 = Lt_ * J.block<6, 3>(0, 6);
      }
    }

    return true;
  }

 private:
  Eigen::Isometry3d Tcb_;
  Eigen::Isometry3d Tvw_;
  Eigen::Isometry3d Tvo_obs_;
  Eigen::Matrix<double, kMeasDim, kMeasDim> Lt_;

  MAPPtr map_ptr_;
};

}  // namespace cg