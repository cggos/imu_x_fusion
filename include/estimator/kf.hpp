#pragma once

#include "estimator/estimator.hpp"
#include "fusion/predictor.hpp"
#include "fusion/updator.hpp"

namespace cg {

class KF : public StateEstimator, public Predictor, public Updator {
 public:
  KF() {}

  KF(double acc_n, double gyr_n, double acc_w, double gyr_w) : Predictor(state_ptr_, acc_n, gyr_n, acc_w, gyr_w) {}

  virtual void predict(ImuDataConstPtr last_imu, ImuDataConstPtr curr_imu) = 0;

  // virtual void update() = 0;

  virtual ~KF() {}

 public:
  Eigen::MatrixXd measurement_cov_;
  Eigen::MatrixXd measurement_noise_cov_;
};
using KFPtr = std::unique_ptr<KF>;

}  // namespace cg