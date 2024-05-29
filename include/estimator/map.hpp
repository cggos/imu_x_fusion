#pragma once

#include "estimator/estimator.hpp"
#include "fusion/predictor.hpp"

namespace cg {

class MAP : public StateEstimator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<MAP>;

  MAP() = default;

  void predict(Predictor::Data::ConstPtr data_ptr) { predictor_ptr_->process(data_ptr); }

  virtual ~MAP() {}

 public:
  Predictor::Ptr predictor_ptr_;
};

}  // namespace cg