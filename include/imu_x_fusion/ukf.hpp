#pragma once

#include <sensor_msgs/Imu.h>

#include <deque>

#include "imu_x_fusion/types.hpp"
#include "imu_x_fusion/utils.h"

namespace cg {

ANGULAR_ERROR State::kAngError = ANGULAR_ERROR::LOCAL_ANGULAR_ERROR;  // TODO

constexpr int kStateDimAug = kStateDim + kNoiseDim;
constexpr int kSigmaPointsNum = 2 * kStateDimAug + 1;

// class StateAug : public State {
//  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//   double acc_noise_;
//   double gyr_noise_;
//   double acc_bias_noise_;
//   double gyr_bias_noise_;

//   const Eigen::Matrix<double, kStateDimAug, 1> vector_aug(const State &state) const {
//     Eigen::Matrix<double, kStateDimAug, 1> vec;

//     vec.head(kStateDim) = state.vec();
//     vec.segment<3>(int(kStateDim)) = Eigen::Vector3d(acc_noise_);
//     vec.segment<3>(kStateDim + 3) = Eigen::Vector3d(acc_bias_noise_);
//     vec.segment<3>(kStateDim + 6) = Eigen::Vector3d(gyr_noise_);
//     vec.segment<3>(kStateDim + 9) = Eigen::Vector3d(gyr_bias_noise_);

//     return vec;
//   }
// };
// using StateAugPtr = std::shared_ptr<StateAug>;

class UKF {
 public:
  StatePtr state_ptr_;
  // StateAugPtr state_aug_ptr_;

  bool initialized_ = false;
  const int kImuBufSize = 200;
  std::deque<ImuDataConstPtr> imu_buf_;
  ImuDataConstPtr last_imu_ptr_;

  UKF() = delete;

  UKF(const UKF &) = delete;

  explicit UKF(double acc_n = 1e-2, double gyr_n = 1e-4, double acc_w = 1e-6, double gyr_w = 1e-8)
      : acc_noise_(acc_n), gyr_noise_(gyr_n), acc_bias_noise_(acc_w), gyr_bias_noise_(gyr_w) {
    state_ptr_ = std::make_shared<State>();

    // state_aug_ptr_ = std::make_shared<StateAug>();
    // state_aug_ptr_->acc_noise_ = acc_n;
    // state_aug_ptr_->gyr_noise_ = gyr_n;
    // state_aug_ptr_->acc_bias_noise_ = acc_w;
    // state_aug_ptr_->gyr_bias_noise_ = gyr_w;

    Q_ = Eigen::Matrix<double, kNoiseDim, kNoiseDim>::Zero();
    Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * acc_noise_ * acc_noise_;
    Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * gyr_noise_ * gyr_noise_;
    Q_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * acc_bias_noise_ * acc_bias_noise_;
    Q_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * gyr_bias_noise_ * gyr_bias_noise_;

    // weights and scale
    double alpha = 0.001;  // 1 × 10−4 ≤ α ≤ 1
    double beta = 2.;
    double kappa = 3 - kStateDimAug;
    double alpha2 = alpha * alpha;

    double lambda = alpha2 * (kStateDimAug + kappa) - kStateDimAug;  // (alpha2 - 1) * kStateDimAug;

    double n_plus_lambda = kStateDimAug + lambda;

    scale_ = std::sqrt(n_plus_lambda);

    double weight = 0.5 / n_plus_lambda;
    weights_mean_[0] = lambda / n_plus_lambda;
    weights_cov_[0] = weights_mean_[0] + (1 - alpha2 + beta);  // (3 - alpha2)
    for (int i = 1; i < kSigmaPointsNum; i++) weights_mean_[i] = weights_cov_[i] = weight;
  }

  void set_cov(double sigma_p, double sigma_v, double sigma_rp, double sigma_yaw, double sigma_ba, double sigma_bg) {
    state_ptr_->cov.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * sigma_p * sigma_p;      // pos std: sigma_p m
    state_ptr_->cov.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * sigma_v * sigma_v;      // vel std: sigma_v m/s
    state_ptr_->cov.block<2, 2>(6, 6) = Eigen::Matrix2d::Identity() * sigma_rp * sigma_rp;    // roll pitch std
    state_ptr_->cov(8, 8) = sigma_yaw * sigma_yaw;                                            // yaw std
    state_ptr_->cov.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * sigma_ba * sigma_ba;    // Acc bias
    state_ptr_->cov.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * sigma_bg * sigma_bg;  // Gyr bias
  }

  void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
    ImuDataPtr imu_data_ptr = std::make_shared<ImuData>();
    imu_data_ptr->timestamp = imu_msg->header.stamp.toSec();
    imu_data_ptr->acc[0] = imu_msg->linear_acceleration.x;
    imu_data_ptr->acc[1] = imu_msg->linear_acceleration.y;
    imu_data_ptr->acc[2] = imu_msg->linear_acceleration.z;
    imu_data_ptr->gyr[0] = imu_msg->angular_velocity.x;
    imu_data_ptr->gyr[1] = imu_msg->angular_velocity.y;
    imu_data_ptr->gyr[2] = imu_msg->angular_velocity.z;

    // remove spikes
    static Eigen::Vector3d last_am = Eigen::Vector3d::Zero();
    if (imu_data_ptr->acc.norm() > 5 * kG) {
      imu_data_ptr->acc = last_am;
    } else {
      last_am = imu_data_ptr->acc;
    }

    if (!initialized_) {
      imu_buf_.push_back(imu_data_ptr);
      if (imu_buf_.size() > kImuBufSize) imu_buf_.pop_front();
      return;
    }

    predict(last_imu_ptr_, imu_data_ptr);

    last_imu_ptr_ = imu_data_ptr;

    // imu_buf_.push_back(imu_data_ptr);
  }

  void predict(ImuDataConstPtr last_imu, ImuDataConstPtr curr_imu) {
    const double dt = curr_imu->timestamp - last_imu->timestamp;
    const double dt2 = dt * dt;

    // compute sigma points
    Eigen::MatrixXd sp_mat_aug = Eigen::MatrixXd::Zero(kStateDimAug, kSigmaPointsNum);
    Eigen::VectorXd x_aug = Eigen::VectorXd::Zero(kStateDimAug);
    Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(kStateDimAug, kStateDimAug);

    x_aug.head(kStateDim) = predicted_x_;
    // x_aug.segment<3>(kStateDim + 0) = acc_noise_ * Eigen::Vector3d::Identity();
    // x_aug.segment<3>(kStateDim + 3) = gyr_noise_ * Eigen::Vector3d::Identity();
    // x_aug.segment<3>(kStateDim + 6) = acc_bias_noise_ * Eigen::Vector3d::Identity();
    // x_aug.segment<3>(kStateDim + 9) = gyr_bias_noise_ * Eigen::Vector3d::Identity();

    P_aug.topLeftCorner(kStateDim, kStateDim) = predicted_P_;
    P_aug.bottomRightCorner(kNoiseDim, kNoiseDim) = Q_;

    const Eigen::MatrixXd &L = P_aug.llt().matrixL();

    sp_mat_aug.col(0) = x_aug;
    for (int c = 0; c < kStateDimAug; c++) {
      const int i = c + 1;
      sp_mat_aug.col(i) = x_aug + scale_ * L.col(c);
      sp_mat_aug.col(i + kStateDimAug) = x_aug - scale_ * L.col(c);
    }

    // predict sigma points
    predicted_sp_mat_ = Eigen::MatrixXd::Zero(kStateDim, kSigmaPointsNum);
    for (int i = 0; i < kSigmaPointsNum; i++) {
      const Eigen::VectorXd &sp_aug = sp_mat_aug.col(i);
      State last_state, state;
      last_state.from_vec(sp_aug.head(kStateDim));
      const auto &vec_na = sp_aug.segment<3>(kStateDim + 0);
      const auto &vec_ng = sp_aug.segment<3>(kStateDim + 3);
      const auto &vec_wa = sp_aug.segment<3>(kStateDim + 6);
      const auto &vec_wg = sp_aug.segment<3>(kStateDim + 9);
      {
        const Eigen::Vector3d acc_unbias = 0.5 * (last_imu->acc + curr_imu->acc) - last_state.acc_bias + vec_wa;
        const Eigen::Vector3d gyr_unbias = 0.5 * (last_imu->gyr + curr_imu->gyr) - last_state.gyr_bias + vec_wg;
        const Eigen::Vector3d acc_nominal = last_state.r_GI * acc_unbias + Eigen::Vector3d(0, 0, -kG);
        state.p_GI = last_state.p_GI + last_state.v_GI * dt + 0.5 * acc_nominal * dt2;
        state.v_GI = last_state.v_GI + acc_nominal * dt;
        state.r_GI = State::rotation_update(last_state.r_GI, State::delta_rot_mat(gyr_unbias * dt));
        state.acc_bias = last_state.acc_bias + vec_na * dt;
        state.gyr_bias = last_state.gyr_bias + vec_ng * dt;
      }
      predicted_sp_mat_.col(i) = state.vec();
    }

    // predict sigma points mean
    predicted_x_ = Eigen::VectorXd::Zero(kStateDim);
    for (int c = 0; c < kSigmaPointsNum; c++) {
      predicted_x_ += weights_mean_[c] * predicted_sp_mat_.col(c);
    }

    // predict sigma points covariance
    predicted_P_ = Eigen::MatrixXd::Zero(kStateDim, kStateDim);
    Eigen::VectorXd dx = Eigen::VectorXd(kStateDim);
    for (int c = 0; c < kSigmaPointsNum; c++) {
      dx = predicted_sp_mat_.col(c) - predicted_x_;
      predicted_P_ += weights_cov_[c] * dx * dx.transpose();
    }

    state_ptr_->timestamp = curr_imu->timestamp;
    state_ptr_->from_vec(predicted_x_);
    state_ptr_->cov = predicted_P_;
  }

  bool init_rot_from_imudata(Eigen::Matrix3d &r_GI, const std::deque<ImuDataConstPtr> &imu_buf) {
    // mean and std of IMU accs
    Eigen::Vector3d sum_acc(0., 0., 0.);
    for (const auto imu_data : imu_buf) {
      sum_acc += imu_data->acc;
    }
    const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buf.size();
    printf("[cggos %s] mean_acc: (%f, %f, %f)!!!\n", __FUNCTION__, mean_acc[0], mean_acc[1], mean_acc[2]);

    Eigen::Vector3d sum_err2(0., 0., 0.);
    for (const auto imu_data : imu_buf) sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
    const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buf.size()).cwiseSqrt();

    // acc std limit: 3
    if (std_acc.maxCoeff() > 3.0) {
      printf("[cggos %s] Too big acc std: (%f, %f, %f)!!!\n", __FUNCTION__, std_acc[0], std_acc[1], std_acc[2]);
      return false;
    }

    // Compute rotation.
    // ref: https://github.com/rpng/open_vins/blob/master/ov_core/src/init/InertialInitializer.cpp

    // Three axises of the ENU frame in the IMU frame.
    // z-axis
    const Eigen::Vector3d &z_axis = mean_acc.normalized();

    // x-axis
    Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
    x_axis.normalize();

    // y-axis
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();

    Eigen::Matrix3d r_IG;
    r_IG.block<3, 1>(0, 0) = x_axis;
    r_IG.block<3, 1>(0, 1) = y_axis;
    r_IG.block<3, 1>(0, 2) = z_axis;

    r_GI = r_IG.transpose();

    initialized_ = true;

    return true;
  }

  ~UKF() {}

 private:
  double acc_noise_;
  double gyr_noise_;
  double acc_bias_noise_;
  double gyr_bias_noise_;
  Eigen::Matrix<double, kNoiseDim, kNoiseDim> Q_;

 public:
  double scale_;
  double weights_mean_[kSigmaPointsNum];
  double weights_cov_[kSigmaPointsNum];

  Eigen::VectorXd predicted_x_;
  Eigen::MatrixXd predicted_P_;
  Eigen::MatrixXd predicted_sp_mat_;
};

using UKFPtr = std::unique_ptr<UKF>;

}  // namespace cg