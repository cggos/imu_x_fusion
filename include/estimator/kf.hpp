#pragma once

#include "estimator/estimator.hpp"
#include "fusion/predictor.hpp"

namespace cg {

class KF : public StateEstimator, public Predictor {
 public:
  KF() {}

  KF(double acc_n, double gyr_n, double acc_w, double gyr_w) : Predictor(state_ptr_, acc_n, gyr_n, acc_w, gyr_w) {}

  virtual void predict(ImuDataConstPtr last_imu, ImuDataConstPtr curr_imu) = 0;

  // virtual void update();

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

  virtual ~KF() {}

 public:
  Eigen::MatrixXd measurement_cov_;
  Eigen::MatrixXd measurement_noise_cov_;
};
using KFPtr = std::unique_ptr<KF>;

}  // namespace cg