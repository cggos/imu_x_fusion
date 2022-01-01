#pragma once

#include "imu_x_fusion/types.hpp"
#include "imu_x_fusion/utils.h"

namespace cg {

ANGULAR_ERROR State::kAngError = ANGULAR_ERROR::LOCAL_ANGULAR_ERROR;

class KF {
 public:
  StatePtr state_ptr_;
  StatePtr state_ptr_i_;  // for IEKF

  const JACOBIAN_MEASUREMENT kJacobMeasurement_ = JACOBIAN_MEASUREMENT::HX_X;

  bool initialized_ = false;
  const int kImuBufSize = 200;
  std::deque<ImuDataConstPtr> imu_buf_;
  ImuDataConstPtr last_imu_ptr_;

  KF() = delete;

  KF(const KF &) = delete;

  explicit KF(double acc_n = 1e-2, double gyr_n = 1e-4, double acc_w = 1e-6, double gyr_w = 1e-8)
      : acc_noise_(acc_n), gyr_noise_(gyr_n), acc_bias_noise_(acc_w), gyr_bias_noise_(gyr_w) {
    state_ptr_ = std::make_shared<State>();
    state_ptr_i_ = std::make_shared<State>();

    const double sigma_rp = 10. * kDegreeToRadian;
    const double sigma_yaw = 100. * kDegreeToRadian;
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

  /**
   * @brief predict procedure
   * @param last_imu
   * @param curr_imu
   */
  void predict(ImuDataConstPtr last_imu, ImuDataConstPtr curr_imu) {
    const double dt = curr_imu->timestamp - last_imu->timestamp;
    const double dt2 = dt * dt;

    State last_state = *state_ptr_;

    // timestamp
    state_ptr_->timestamp = curr_imu->timestamp;

    //
    // ESKF 5.4.1 The nominal state kinematics
    //

    // p v R
    const Eigen::Vector3d acc_unbias = 0.5 * (last_imu->acc + curr_imu->acc) - last_state.acc_bias;
    const Eigen::Vector3d gyr_unbias = 0.5 * (last_imu->gyr + curr_imu->gyr) - last_state.gyr_bias;
    const Eigen::Vector3d acc_nominal = last_state.r_GI * acc_unbias + Eigen::Vector3d(0, 0, -kG);
    state_ptr_->p_GI = last_state.p_GI + last_state.v_GI * dt + 0.5 * acc_nominal * dt2;
    state_ptr_->v_GI = last_state.v_GI + acc_nominal * dt;
    const auto &dR = State::delta_rot_mat(gyr_unbias * dt);
    state_ptr_->r_GI = State::rotation_update(last_state.r_GI, dR);

    //
    // ESKF
    // 5.4.3 (local angular error)
    // 7.2.3 (global angular error)
    // The error-state Jacobian and perturbation matrices
    //

    // Fx
    MatrixSD Fx = MatrixSD::Identity();
    Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    Fx.block<3, 3>(3, 6) = -state_ptr_->r_GI * cg::skew_matrix(acc_unbias) * dt;
    Fx.block<3, 3>(3, 9) = -state_ptr_->r_GI * dt;
    if (State::kAngError == ANGULAR_ERROR::LOCAL_ANGULAR_ERROR) {
      Fx.block<3, 3>(6, 6) = dR.transpose();
      Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;
    } else {
      Fx.block<3, 3>(6, 12) = -state_ptr_->r_GI * dt;
    }

    // Fi
    Eigen::Matrix<double, kStateDim, kNoiseDim> Fi = Eigen::Matrix<double, kStateDim, kNoiseDim>::Zero();
    Fi.block<12, kNoiseDim>(3, 0) = Eigen::Matrix<double, 12, kNoiseDim>::Identity();

    // Qi
    Eigen::Matrix<double, kNoiseDim, kNoiseDim> Qi = Eigen::Matrix<double, kNoiseDim, kNoiseDim>::Zero();
    Qi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dt2 * acc_noise_;
    Qi.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * dt2 * gyr_noise_;
    Qi.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * dt * acc_bias_noise_;
    Qi.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * dt * gyr_bias_noise_;

    // P: error-state covariance
    state_ptr_->cov = Fx * last_state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();
  }

  template <class H_type, class R_type, class K_type>
  void update_K(const Eigen::MatrixBase<H_type> &H, const Eigen::MatrixBase<R_type> &R, Eigen::MatrixBase<K_type> &K) {
    const MatrixSD &P = state_ptr_->cov;
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

  ~KF() {}

 private:
  double acc_noise_;
  double gyr_noise_;
  double acc_bias_noise_;
  double gyr_bias_noise_;
};

using KFPtr = std::unique_ptr<KF>;

}  // namespace cg