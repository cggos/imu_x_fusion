#pragma once

#include <sensor_msgs/Imu.h>

#include "common/state.hpp"
#include "sensor/imu.hpp"

namespace cg {

class Predictor {
 public:
  Predictor() {}

  Predictor(const Predictor &) = delete;

  Predictor(StatePtr &state_ptr, double acc_n, double gyr_n, double acc_w, double gyr_w)
      : state_p_(state_ptr), imu_model_(acc_n, gyr_n, acc_w, gyr_w) {}

  bool init(double ts_meas) { return inited_ = imu_model_.init(*state_p_, ts_meas, last_imu_ptr_); }

  void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
    ImuDataPtr imu_data_ptr = std::make_shared<ImuData>();
    imu_data_ptr->timestamp = imu_msg->header.stamp.toSec();
    imu_data_ptr->acc[0] = imu_msg->linear_acceleration.x;
    imu_data_ptr->acc[1] = imu_msg->linear_acceleration.y;
    imu_data_ptr->acc[2] = imu_msg->linear_acceleration.z;
    imu_data_ptr->gyr[0] = imu_msg->angular_velocity.x;
    imu_data_ptr->gyr[1] = imu_msg->angular_velocity.y;
    imu_data_ptr->gyr[2] = imu_msg->angular_velocity.z;

    if (!imu_model_.push_data(imu_data_ptr, inited_)) return;

    predict(last_imu_ptr_, imu_data_ptr);

    last_imu_ptr_ = imu_data_ptr;
  }

  virtual void predict(ImuDataConstPtr last_imu, ImuDataConstPtr curr_imu) = 0;

  virtual ~Predictor() {}

 public:
  bool inited_ = false;

  IMU imu_model_;
  ImuDataConstPtr last_imu_ptr_;

 private:
  StatePtr state_p_;
};
using PredictorPtr = std::unique_ptr<Predictor>;

}  // namespace cg