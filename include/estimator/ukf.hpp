#pragma once

#include "estimator/kf.hpp"

namespace cg {

constexpr int kStateDimAug = kStateDim + kNoiseDim;

class UKF : public KF {
 public:
  UKF() = delete;

  UKF(const UKF &) = delete;

  explicit UKF(double acc_n = 1e-2, double gyr_n = 1e-4, double acc_w = 1e-6, double gyr_w = 1e-8)
      : KF(acc_n, gyr_n, acc_w, gyr_w) {
    sigma_points_num_ = is_Q_aug_ ? 2 * kStateDimAug + 1 : 2 * kStateDim + 1;

    weights_mean_.resize(sigma_points_num_);
    weights_cov_.resize(sigma_points_num_);

    // weights and scale
    int N = is_Q_aug_ ? kStateDimAug : kStateDim;
    double alpha = 0.001;  // 1 × 10−4 ≤ α ≤ 1
    double beta = 2.;
    double kappa = 0;  // 0. or 3 - N;
    double alpha2 = alpha * alpha;

    double lambda = alpha2 * (N + kappa) - N;

    double n_plus_lambda = N + lambda;

    scale_ = std::sqrt(n_plus_lambda);

    weights_mean_[0] = lambda / n_plus_lambda;
    weights_cov_[0] = weights_mean_[0] + (1 - alpha2 + beta);
    double weight = 0.5 / n_plus_lambda;
    for (int i = 1; i < sigma_points_num_; i++) weights_mean_[i] = weights_cov_[i] = weight;
  }

  void predict(ImuDataConstPtr last_imu, ImuDataConstPtr curr_imu) {
    const double dt = curr_imu->timestamp - last_imu->timestamp;

    // compute sigma points
    int N0 = is_Q_aug_ ? kStateDimAug : kStateDim;
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(N0);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(N0, N0);
    Eigen::MatrixXd sp_mat0 = Eigen::MatrixXd::Zero(N0, sigma_points_num_);

    x0.head(kStateDim) = predicted_x_;

    P0.topLeftCorner(kStateDim, kStateDim) = predicted_P_;
    if (is_Q_aug_) P0.bottomRightCorner(kNoiseDim, kNoiseDim) = imu_model_.noise_cov(dt);  // TODO: Q with dt or not ?

    Eigen::MatrixXd L;
    if (!is_SVD_P_)
      L = P0.llt().matrixL();
    else {
      // for not-PSD cov matrix
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(P0, Eigen::ComputeFullU | Eigen::ComputeFullV);
      const auto &S = svd.matrixU().inverse() * P0 * svd.matrixV().transpose().inverse();  // S = U^-1 * A * VT * -1
      const Eigen::MatrixXd &Smat = S.matrix();
      const Eigen::MatrixXd &S_sqr = Smat.llt().matrixL();
      L = svd.matrixU().matrix() * S_sqr;  // U * sqrt(S)
    }

    sp_mat0.col(0) = x0;
    for (int c = 0; c < N0; c++) {
      const int i = c + 1;
      sp_mat0.col(i) = x0 + scale_ * L.col(c);
      sp_mat0.col(i + N0) = x0 - scale_ * L.col(c);
    }

    // predict sigma points
    predicted_sp_mat_ = Eigen::MatrixXd::Zero(kStateDim, sigma_points_num_);
    for (int i = 0; i < sigma_points_num_; i++) {
      const Eigen::VectorXd &sp = sp_mat0.col(i);
      State last_state, state;
      last_state.from_vec(sp.head(kStateDim));
      if (is_Q_aug_)
        imu_model_.propagate_state(last_imu, curr_imu, last_state, state, true, false,
                                   sp.segment<kNoiseDim>(kStateDim));
      else
        imu_model_.propagate_state(last_imu, curr_imu, last_state, state, true, true);

      predicted_sp_mat_.col(i) = state.vec();
    }

    // predict sigma points mean
    predicted_x_ = Eigen::VectorXd::Zero(kStateDim);
    for (int c = 0; c < sigma_points_num_; c++) {
      predicted_x_ += weights_mean_[c] * predicted_sp_mat_.col(c);
    }

    // predict sigma points covariance
    if (!is_JUKF_) {
      predicted_P_ = Eigen::MatrixXd::Zero(kStateDim, kStateDim);
      Eigen::VectorXd dx = Eigen::VectorXd(kStateDim);
      for (int c = 0; c < sigma_points_num_; c++) {
        dx = predicted_sp_mat_.col(c) - predicted_x_;
        predicted_P_ += weights_cov_[c] * dx * dx.transpose();
      }
      predicted_P_.noalias() += imu_model_.noise_cov_discret_time(dt);  // will diverge if not add Q
    } else {
      imu_model_.propagate_state_cov(last_imu, curr_imu, *state_ptr_, *state_ptr_);
      predicted_P_ = state_ptr_->cov;
    }
    predicted_P_ = 0.5 * (predicted_P_ + predicted_P_.transpose());

    // update state
    state_ptr_->timestamp = curr_imu->timestamp;
    state_ptr_->from_vec(predicted_x_);
    state_ptr_->cov = predicted_P_;
  }

  ~UKF() {}

 private:
  bool is_Q_aug_ = true;   // P  <--  [P Q]  or  P + Q
  bool is_SVD_P_ = false;  // LLT or SVD for P0 (for not-PSD cov matrix)
  bool is_JUKF_ = false;   // same with EKF for cov propagation or not

 public:
  double scale_;
  int sigma_points_num_;
  std::vector<double> weights_mean_;
  std::vector<double> weights_cov_;

  Eigen::VectorXd predicted_x_;
  Eigen::MatrixXd predicted_P_;
  Eigen::MatrixXd predicted_sp_mat_;
};

using UKFPtr = std::unique_ptr<UKF>;

}  // namespace cg