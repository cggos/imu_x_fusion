#pragma once

#include <boost/math/distributions/chi_squared.hpp>

#include "estimator/kf.hpp"
#include "sensor/odom_6dof.hpp"

namespace cg {

constexpr int kStateDimAug = kStateDim + kNoiseDim;

class UKF : public KF {
 public:
  UKF() = delete;

  UKF(const UKF &) = delete;

  explicit UKF(double acc_n = 1e-2, double gyr_n = 1e-4, double acc_w = 1e-6, double gyr_w = 1e-8)
      : KF(acc_n, gyr_n, acc_w, gyr_w) {
    sigma_points_num_ = is_Q_aug_ ? 2 * kStateDimAug + 1 : 2 * kStateDim + 1;

    double weight_m0, weight_c0, weight_i;
    compute_scale_weights(scale_, weight_m0, weight_c0, weight_i);

    weights_m_.resize(sigma_points_num_);
    weights_c_.resize(sigma_points_num_);
    weights_m_[0] = weight_m0;
    weights_c_[0] = weight_c0;
    for (int i = 1; i < sigma_points_num_; i++) weights_m_[i] = weights_c_[i] = weight_i;

    for (int i = 1; i < 100; ++i) {
      boost::math::chi_squared chi_squared_dist(i);
      chi_squared_test_table_[i] = boost::math::quantile(chi_squared_dist, 0.05);
    }
  }

  void predict(ImuDataConstPtr last_imu, ImuDataConstPtr curr_imu) {
    const double dt = curr_imu->timestamp - last_imu->timestamp;

    state_ptr_->timestamp = curr_imu->timestamp;

    const Eigen::MatrixXd &sp_mat0 = generate_sigma_points(dt);

    predict_sigma_points(last_imu, curr_imu, sp_mat0);

    predict_sigma_points_mean_cov(last_imu, curr_imu);
  }

  void update(const Eigen::Isometry3d &Tvw, const Eigen::Isometry3d &Tcb, const Eigen::Isometry3d &Tvo) {
    predict_measurement_sigma_points(Tvw, Tcb);

    predict_measurement_mean_cov();

    const Eigen::MatrixXd &Pxz = cross_correlation_matrix();

    Eigen::MatrixXd K = Pxz * measurement_cov_.inverse();

    Eigen::VectorXd dz;
    {
      Eigen::Matrix<double, kMeasDim, 1> vec_vo;
      vec_vo.segment<3>(0) = Tvo.translation();
      vec_vo.segment<3>(3) = Utils::rot_mat_to_vec(Tvo.linear());
      dz = vec_vo - predicted_z_;
    }

    *state_ptr_ = *state_ptr_ + K * dz;

    Eigen::MatrixXd predicted_P;
    predicted_P = state_ptr_->cov - K * measurement_cov_ * K.transpose();
    predicted_P = 0.5 * (predicted_P + predicted_P.transpose());

    // condition number
    // 解决：因观测误差较大，使P负定，致使后面P的Cholesky分解失败出现NaN，导致滤波器发散
    {
      double cond_num = Utils::condition_number(predicted_P);
      std::cout << "cond num of P: " << cond_num << std::endl;
      if (cond_num > 1e5) predicted_P = predicted_P.diagonal().asDiagonal();
    }
    state_ptr_->cov = predicted_P;
  }

  virtual ~UKF() {}

 private:
  void compute_scale_weights(double &scale, double &weight_m0, double &weight_c0, double &weight_i) {
    int N = is_Q_aug_ ? kStateDimAug : kStateDim;

    double alpha = 0.001;  // 1 × 10−4 ≤ α ≤ 1
    double beta = 2.;
    double kappa = 0;  // 0. or 3 - N;
    double alpha2 = alpha * alpha;

    double lambda = alpha2 * (N + kappa) - N;

    double n_plus_lambda = N + lambda;

    // scale
    scale = std::sqrt(n_plus_lambda);

    // weights
    weight_m0 = lambda / n_plus_lambda;
    weight_c0 = weight_m0 + (1 - alpha2 + beta);
    weight_i = 0.5 / n_plus_lambda;
  }

  Eigen::MatrixXd generate_sigma_points(double dt) {
    int N0 = is_Q_aug_ ? kStateDimAug : kStateDim;

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(N0);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(N0, N0);
    Eigen::MatrixXd sp_mat0 = Eigen::MatrixXd::Zero(N0, sigma_points_num_);

    x0.head(kStateDim) = state_ptr_->vec();

    P0.topLeftCorner(kStateDim, kStateDim) = state_ptr_->cov;
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

    return sp_mat0;
  }

  void predict_sigma_points(ImuDataConstPtr last_imu, ImuDataConstPtr curr_imu, const Eigen::MatrixXd &sp_mat0) {
    predicted_sp_mat_ = Eigen::MatrixXd::Zero(kStateDim, sigma_points_num_);
    for (int i = 0; i < sigma_points_num_; i++) {
      const Eigen::VectorXd &sp = sp_mat0.col(i);
      State last_state, state;
      last_state.from_vec(sp.head(kStateDim));
      if (is_Q_aug_) {
        const Eigen::VectorXd &noise_vec = sp.segment<kNoiseDim>(kStateDim);
        imu_model_.propagate_state(last_imu, curr_imu, last_state, state, true, false, noise_vec);
      } else
        imu_model_.propagate_state(last_imu, curr_imu, last_state, state, true, true);
      predicted_sp_mat_.col(i) = state.vec();
    }
  }

  void predict_sigma_points_mean_cov(ImuDataConstPtr last_imu, ImuDataConstPtr curr_imu) {
    // predict sigma points mean
    Eigen::VectorXd predicted_x = Eigen::VectorXd::Zero(kStateDim);
    for (int c = 0; c < sigma_points_num_; c++) {
      predicted_x += weights_m_[c] * predicted_sp_mat_.col(c);
    }
    state_ptr_->from_vec(predicted_x);

    // predict sigma points covariance
    if (!is_JUKF_) {
      Eigen::MatrixXd predicted_P = Eigen::MatrixXd::Zero(kStateDim, kStateDim);
      Eigen::VectorXd dx = Eigen::VectorXd(kStateDim);
      for (int c = 0; c < sigma_points_num_; c++) {
        dx = predicted_sp_mat_.col(c) - predicted_x;
        predicted_P += weights_c_[c] * dx * dx.transpose();
      }
      const double dt = curr_imu->timestamp - last_imu->timestamp;
      state_ptr_->cov = predicted_P + imu_model_.noise_cov_discret_time(dt);  // will diverge if not add Q
    } else
      imu_model_.propagate_state_cov(last_imu, curr_imu, *state_ptr_, *state_ptr_);
  }

  void predict_measurement_sigma_points(const Eigen::Isometry3d &Tvw, const Eigen::Isometry3d &Tcb) {
    Eigen::Isometry3d Twb;
    predicted_meas_sp_mat_ = Eigen::MatrixXd::Zero(kMeasDim, sigma_points_num_);
    for (int i = 0; i < sigma_points_num_; i++) {
      const auto &sp = predicted_sp_mat_.col(i);

      Twb.translation() = sp.segment<3>(0);
      Twb.linear() = Utils::rot_vec_to_mat(sp.segment<3>(6));

      // measurement estimation h(x), Twb in frame V --> Tc0cn
      const Eigen::Isometry3d &Twb_in_V = Tvw * Twb * Tcb.inverse();

      predicted_meas_sp_mat_.col(i).segment<3>(0) = Twb_in_V.translation();
      predicted_meas_sp_mat_.col(i).segment<3>(3) = Utils::rot_mat_to_vec(Twb_in_V.linear());
    }
  }

  void predict_measurement_mean_cov() {
    // predict measurement mean
    predicted_z_ = Eigen::VectorXd::Zero(kMeasDim);
    for (int c = 0; c < sigma_points_num_; c++) {
      predicted_z_ += weights_m_[c] * predicted_meas_sp_mat_.col(c);
    }

    // predict measurement covariance
    Eigen::MatrixXd Pzz = Eigen::MatrixXd::Zero(kMeasDim, kMeasDim);
    Eigen::VectorXd dz = Eigen::VectorXd(kMeasDim);
    for (int c = 0; c < sigma_points_num_; c++) {
      dz = predicted_meas_sp_mat_.col(c) - predicted_z_;
      Pzz += weights_c_[c] * dz * dz.transpose();
    }
    Pzz += measurement_noise_cov_;
    measurement_cov_ = Pzz;
  }

  Eigen::MatrixXd cross_correlation_matrix() {
    Eigen::VectorXd dx, dz;
    Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(kStateDim, kMeasDim);
    for (int c = 0; c < sigma_points_num_; c++) {
      dx = predicted_sp_mat_.col(c) - state_ptr_->vec();
      dz = predicted_meas_sp_mat_.col(c) - predicted_z_;
      Pxz += weights_c_[c] * dx * dz.transpose();
    }
    return Pxz;
  }

  double chi2(const Eigen::VectorXd &dz) { return dz.transpose() * measurement_cov_.ldlt().solve(dz); }

  bool gating_test(const Eigen::VectorXd &dz) {
    const float scale = 200;
    const int dof = kMeasDim;
    if (chi2(dz) >= scale * chi_squared_test_table_[dof]) {
      return false;
    } else
      return true;
  }

 private:
  bool is_Q_aug_ = true;   // P  <--  [P Q]  or  P + Q
  bool is_SVD_P_ = false;  // LLT or SVD for P0 (for not-PSD cov matrix)
  bool is_JUKF_ = false;   // same with EKF for cov propagation or not

  double scale_;
  int sigma_points_num_;
  std::vector<double> weights_m_;
  std::vector<double> weights_c_;

  Eigen::VectorXd predicted_z_;
  Eigen::MatrixXd predicted_sp_mat_;
  Eigen::MatrixXd predicted_meas_sp_mat_;

  std::map<int, double> chi_squared_test_table_;
};

using UKFPtr = std::unique_ptr<UKF>;

}  // namespace cg