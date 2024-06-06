#pragma once

#include <Eigen/Core>
#include <deque>
#include <memory>

#include "common/state.hpp"
#include "fusion/predictor.hpp"

namespace cg {

class ImuData : public Predictor::Data {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d acc;
  Eigen::Vector3d gyr;

  ImuData(double ts, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
      : Predictor::Data(ts, acc, gyr), acc(acc), gyr(gyr) {}

  using Ptr = std::shared_ptr<ImuData>;
  using ConstPtr = std::shared_ptr<const ImuData>;
};

class IMU : public Predictor {
 public:
  IMU(State::Ptr state_ptr, double acc_n = 1e-2, double gyr_n = 1e-4, double acc_w = 1e-6, double gyr_w = 1e-8)
      : Predictor(state_ptr), acc_noise_(acc_n), gyr_noise_(gyr_n), acc_bias_noise_(acc_w), gyr_bias_noise_(gyr_w) {}

  using Ptr = std::shared_ptr<IMU>;

  virtual void predict(Data::ConstPtr last_ptr, Data::ConstPtr curr_ptr) {
    auto last_imu = last_ptr->data_;
    auto curr_imu = curr_ptr->data_;
    if (last_imu.size() != 6 || curr_imu.size() != 6) return;

    State last_state = *state_ptr_;

    state_ptr_->timestamp = curr_ptr->timestamp_;

    ImuData::Ptr last_imu_ptr = std::make_shared<ImuData>(last_ptr->timestamp_, last_imu.head(3), last_imu.tail(3));
    ImuData::Ptr curr_imu_ptr = std::make_shared<ImuData>(curr_ptr->timestamp_, curr_imu.head(3), curr_imu.tail(3));

    propagate_state(last_imu_ptr, curr_imu_ptr, last_state, *state_ptr_);
    propagate_state_cov(last_imu_ptr, curr_imu_ptr, last_state, *state_ptr_);
  }

  virtual bool push_data(Data::ConstPtr data_ptr) {
    auto acc = data_ptr->data_.head(3);
    auto gyr = data_ptr->data_.tail(3);

    auto imu_ptr = std::make_shared<ImuData>(data_ptr->timestamp_, acc, gyr);

    // remove spikes
    static Eigen::Vector3d last_am = Eigen::Vector3d::Zero();
    if (imu_ptr->acc.norm() > 5 * kG) {
      imu_ptr->acc = last_am;
    } else {
      last_am = imu_ptr->acc;
    }

    imu_buf_.push_back(imu_ptr);
    if (imu_buf_.size() > kImuBufSize) imu_buf_.pop_front();

    return true;
  }

  virtual bool init(double ts_meas) {
    if (imu_buf_.size() < kImuBufSize) {
      printf("[cggos %s] ERROR: Not Enough IMU data for Initialization!!!\n", __FUNCTION__);
      return false;
    }

    const auto &imu = imu_buf_.back();
    last_data_ptr_ = std::make_shared<Predictor::Data>(imu->timestamp_, imu->acc, imu->gyr);

    if (std::abs(ts_meas - last_data_ptr_->timestamp_) > 0.05) {
      printf("[cggos %s] ERROR: timestamps are not synchronized!!!\n", __FUNCTION__);
      return false;
    }

    state_ptr_->timestamp = last_data_ptr_->timestamp_;

    inited_ = init_rot_from_imudata(imu_buf_, state_ptr_->Rwb_);

    return inited_;
  }

  /**
   * @brief
   *
   * @ref ESKF 5.4.1 The nominal state kinematics (without noise)
   *
   * @param last_imu
   * @param curr_imu
   * @param last_state
   * @param state
   * @param vec_na
   * @param vec_ng
   * @param vec_wa
   * @param vec_wg
   */
  void propagate_state(
      ImuData::ConstPtr last_imu,
      ImuData::ConstPtr curr_imu,
      const State &last_state,
      State &state,
      bool with_noise = false,
      bool with_const_noise = true,
      const Eigen::Matrix<double, kNoiseDim, 1> &vec_noise = Eigen::Matrix<double, kNoiseDim, 1>::Zero()) {
    const double dt = curr_imu->timestamp_ - last_imu->timestamp_;
    const double dt2 = dt * dt;

    Eigen::Vector3d vec_na = Eigen::Vector3d::Zero();
    Eigen::Vector3d vec_ng = Eigen::Vector3d::Zero();
    Eigen::Vector3d vec_wa = Eigen::Vector3d::Zero();
    Eigen::Vector3d vec_wg = Eigen::Vector3d::Zero();
    if (with_noise) {
      // TODO: check vec_noise empty or not
      if (with_const_noise) {
        vec_na = Eigen::Vector3d(acc_noise_, acc_noise_, acc_noise_);
        vec_ng = Eigen::Vector3d(gyr_noise_, gyr_noise_, gyr_noise_);
        vec_wa = Eigen::Vector3d(acc_bias_noise_, acc_bias_noise_, acc_bias_noise_);
        vec_wg = Eigen::Vector3d(gyr_bias_noise_, gyr_bias_noise_, gyr_bias_noise_);
      } else {
        vec_na = vec_noise.segment<3>(0);
        vec_ng = vec_noise.segment<3>(3);
        vec_wa = vec_noise.segment<3>(6);
        vec_wg = vec_noise.segment<3>(9);
      }
    }

    const Eigen::Vector3d acc_unbias = 0.5 * (last_imu->acc + curr_imu->acc) - last_state.acc_bias - vec_na;
    const Eigen::Vector3d gyr_unbias = 0.5 * (last_imu->gyr + curr_imu->gyr) - last_state.gyr_bias - vec_ng;

    const Eigen::Vector3d acc_nominal = last_state.Rwb_ * acc_unbias + Eigen::Vector3d(0, 0, -kG);
    const auto &dR = State::delta_rot_mat(gyr_unbias * dt);

    state.p_wb_ = last_state.p_wb_ + last_state.v_wb_ * dt + 0.5 * acc_nominal * dt2;
    state.v_wb_ = last_state.v_wb_ + acc_nominal * dt;
    state.Rwb_ = State::rotation_update(last_state.Rwb_, dR);
    state.acc_bias = last_state.acc_bias + vec_wa * dt;
    state.gyr_bias = last_state.gyr_bias + vec_wg * dt;
  }

  /**
   * @brief
   *
   * @ref ESKF
   *      5.4.3 (local angular error)
   *      7.2.3 (global angular error)
   *      The error-state Jacobian and perturbation matrices
   *
   *
   * @param last_imu
   * @param curr_imu
   * @param last_state
   * @param state
   */
  void propagate_state_cov(ImuData::ConstPtr last_imu,
                           ImuData::ConstPtr curr_imu,
                           const State &last_state,
                           State &state) {
    const double dt = curr_imu->timestamp_ - last_imu->timestamp_;

    const Eigen::Vector3d acc_unbias = 0.5 * (last_imu->acc + curr_imu->acc) - last_state.acc_bias;
    const Eigen::Vector3d gyr_unbias = 0.5 * (last_imu->gyr + curr_imu->gyr) - last_state.gyr_bias;

    const auto &dR = State::delta_rot_mat(gyr_unbias * dt);

    // Fx
    MatrixSD Fx = MatrixSD::Identity();
    Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    Fx.block<3, 3>(3, 6) = -state.Rwb_ * Utils::skew_matrix(acc_unbias) * dt;
    Fx.block<3, 3>(3, 9) = -state.Rwb_ * dt;
    if (State::kAngError == ANGULAR_ERROR::LOCAL_ANGULAR_ERROR) {
      Fx.block<3, 3>(6, 6) = dR.transpose();
      Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;
    } else {
      Fx.block<3, 3>(6, 12) = -state.Rwb_ * dt;
    }

    // P: error-state covariance
    state.cov = Fx * last_state.cov * Fx.transpose() + noise_cov_discret_time(dt);
  }

  Eigen::Matrix<double, kNoiseDim, kNoiseDim> noise_cov(double dt) {
    const double dt2 = dt * dt;
    Eigen::Matrix<double, kNoiseDim, kNoiseDim> Qi = Eigen::Matrix<double, kNoiseDim, kNoiseDim>::Zero();
    Qi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dt2 * acc_noise_;
    Qi.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * dt2 * gyr_noise_;
    Qi.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * dt * acc_bias_noise_;
    Qi.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * dt * gyr_bias_noise_;
    return Qi;
  }

  Eigen::Matrix<double, kStateDim, kStateDim> noise_cov_discret_time(double dt) {
    Eigen::Matrix<double, kStateDim, kNoiseDim> Fi = Eigen::Matrix<double, kStateDim, kNoiseDim>::Zero();
    Fi.block<12, kNoiseDim>(3, 0) = Eigen::Matrix<double, 12, kNoiseDim>::Identity();
    return Fi * noise_cov(dt) * Fi.transpose();
  }

  static bool init_rot_from_imudata(const std::deque<ImuData::ConstPtr> &imu_buf, Eigen::Matrix3d &Rwb) {
    // mean and std of IMU accs
    Eigen::Vector3d sum_acc(0., 0., 0.);
    for (const auto &imu_data : imu_buf) sum_acc += imu_data->acc;
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

    if (std_acc.isZero(std::numeric_limits<float>::epsilon())) {
      printf("[cggos %s] acc std is Zero: (%f, %f, %f)!!!\n", __FUNCTION__, std_acc[0], std_acc[1], std_acc[2]);
      return false;
    }

    // Compute rotation
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

    Eigen::Matrix3d Rbw;
    Rbw.block<3, 1>(0, 0) = x_axis;
    Rbw.block<3, 1>(0, 1) = y_axis;
    Rbw.block<3, 1>(0, 2) = z_axis;

    Rwb = Rbw.transpose();

    return true;
  }

 private:
  double acc_noise_;
  double gyr_noise_;
  double acc_bias_noise_;
  double gyr_bias_noise_;

  static const int kImuBufSize = 200;
  std::deque<ImuData::ConstPtr> imu_buf_;
};

}  // namespace cg
