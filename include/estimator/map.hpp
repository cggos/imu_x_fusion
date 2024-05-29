#pragma once

#include "estimator/estimator.hpp"
#include "fusion/predictor.hpp"

namespace cg {

class MAP : public StateEstimator {
 public:
  MAP() = default;

  void predict(Predictor::DataConstPtr data_ptr) { predictor_ptr_->process(data_ptr); }

  virtual ~MAP() {}

 public:
  PredictorPtr predictor_ptr_;
};

using MAPPtr = std::shared_ptr<MAP>;

}  // namespace cg