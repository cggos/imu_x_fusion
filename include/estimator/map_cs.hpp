#pragma once

#include <ceres/ceres.h>

#include "common/state.hpp"
#include "imu_x_fusion/odom_6dof.hpp"

namespace cg {

class MAPCostFunctor : public ceres::SizedCostFunction<6, 15> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MAPCostFunctor(const Eigen::Isometry3d& Tcb, const Eigen::Isometry3d& Tvw, const Eigen::Isometry3d& TvoB) {
    Tcb_ = Tcb;
    Tvw_ = Tvw;
    Tvo_obs_ = TvoB;
  }

  virtual ~MAPCostFunctor() {}

  virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    Eigen::Map<const Eigen::Matrix<double, kStateDim, 1>> state_vec(*parameters);

    State state;
    state.from_vec(state_vec);

    // x_i
    Eigen::Isometry3d Twb_i = state.pose();

    // measurement estimation h(x_i), Twb in frame V --> Tc0cn
    const Eigen::Isometry3d& Twb_in_V = Tvw_ * state.pose() * Tcb_.inverse();

    // residual = z - h(x_i)
    Eigen::Matrix<double, 6, 1> residual;
    residual.topRows(3) = Tvo_obs_.translation() - Twb_in_V.translation();
    residual.bottomRows(3) = State::rotation_residual(Tvo_obs_.linear(), Twb_in_V.linear());

    for (int i = 0; i < 6; i++) residuals[i] = residual[i];

    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor>> J(jacobians[0]);

        const auto& vo_q = Eigen::Quaterniond(Tvo_obs_.linear());
        J = -1.0 * Odom6Dof::measurementH(vo_q, Twb_i, Tvw_, Tcb_);
      }
    }

    return true;
  }

 private:
  Eigen::Isometry3d Tcb_;
  Eigen::Isometry3d Tvw_;
  Eigen::Isometry3d Tvo_obs_;
};

}  // namespace cg