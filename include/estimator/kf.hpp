#pragma once

#include "estimator/estimator.hpp"
#include "fusion/observer.hpp"
#include "fusion/predictor.hpp"

namespace cg {

class KF : public StateEstimator {
 public:
  KF() = default;

  virtual void predict(Predictor::Data::ConstPtr data_ptr) = 0;

  // virtual void update() = 0;

  virtual ~KF() {}

 public:
  Predictor::Ptr predictor_ptr_;
  Observer::Ptr observer_ptr_;

  Eigen::MatrixXd measurement_cov_;
  Eigen::MatrixXd measurement_noise_cov_;
};
using KFPtr = std::unique_ptr<KF>;

}  // namespace cg