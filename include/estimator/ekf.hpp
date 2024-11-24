#pragma once

#include "common/utils.hpp"
#include "estimator/kf.hpp"

namespace cg {

class EKF : public KF {
 public:
  EKF() = default;

  EKF(const EKF &) = delete;

  explicit EKF(double sigma_p, double sigma_v, double sigma_rp, double sigma_yaw, double sigma_ba, double sigma_bg) {
    state_ptr_i_ = std::make_shared<State>();

    state_ptr_->set_cov(sigma_p, sigma_v, sigma_rp, sigma_yaw, sigma_ba, sigma_bg);
  }

  virtual ~EKF() {}

  virtual void predict(Predictor::Data::ConstPtr data_ptr) { predictor_ptr_->process(data_ptr); }

  template <class H_type, class R_type, class K_type>
  void update_K(const Eigen::MatrixBase<H_type> &H, const Eigen::MatrixBase<R_type> &R, Eigen::MatrixBase<K_type> &K) {
    const auto &P = state_ptr_->cov;
    const R_type &S = H * P * H.transpose() + R;
    K = P * H.transpose() * S.inverse();
    // const Eigen::Matrix<double, kStateDim, R_type::RowsAtCompileTime> K = P * H.transpose() * S.inverse();
  }

  template <class H_type, class R_type, class K_type>
  void update_P(const Eigen::MatrixBase<H_type> &H,
                const Eigen::MatrixBase<R_type> &R,
                const Eigen::MatrixBase<K_type> &K) {
    const MatrixSD &I_KH = MatrixSD::Identity() - K * H;
    state_ptr_->cov = I_KH * state_ptr_->cov * I_KH.transpose() + K * R * K.transpose();
  }

 public:
  State::Ptr state_ptr_i_;  // for IEKF
};

using EKFPtr = std::unique_ptr<EKF>;

}  // namespace cg