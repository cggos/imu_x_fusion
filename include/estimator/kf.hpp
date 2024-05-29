#pragma once

#include "estimator/estimator.hpp"
#include "fusion/observer.hpp"
#include "fusion/predictor.hpp"

namespace cg {

class KF : public StateEstimator {
 public:
  KF() = default;

  virtual void predict(Predictor::DataConstPtr data_ptr) = 0;

  // virtual void update() = 0;

  virtual ~KF() {}

 public:
  PredictorPtr predictor_ptr_;
  ObserverPtr observer_ptr_;

  Eigen::MatrixXd measurement_cov_;
  Eigen::MatrixXd measurement_noise_cov_;
};
using KFPtr = std::unique_ptr<KF>;

}  // namespace cg