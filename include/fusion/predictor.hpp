#pragma once

#include "common/state.hpp"

namespace cg {

class Predictor {
 public:
  class Data {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double timestamp_;
    Eigen::VectorXd data_;

    Data(double ts, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr) : timestamp_(ts) {
      data_ = Eigen::MatrixXd::Zero(6, 1);
      data_.head(3) = acc;
      data_.tail(3) = gyr;
    }
  };
  using DataPtr = std::shared_ptr<Data>;
  using DataConstPtr = std::shared_ptr<const Data>;

  Predictor() = default;

  Predictor(const Predictor &) = delete;

  Predictor(StatePtr state_ptr) : state_ptr_(state_ptr) {}

  virtual ~Predictor() {}

  virtual bool init(double ts_meas) = 0;

  void process(DataConstPtr data_ptr, std::function<void(DataConstPtr, DataConstPtr)> func_predict = nullptr) {
    if (!data_ptr) return;

    if (!push_data(data_ptr)) return;

    if (func_predict != nullptr)
      func_predict(last_data_ptr_, data_ptr);
    else
      predict(last_data_ptr_, data_ptr);

    last_data_ptr_ = data_ptr;
  }

 protected:
  virtual bool push_data(DataConstPtr data_ptr) = 0;

  virtual void predict(DataConstPtr last_ptr, DataConstPtr curr_ptr) = 0;

 public:
  bool inited_ = false;

  StatePtr state_ptr_;

 protected:
  DataConstPtr last_data_ptr_;
};
using PredictorPtr = std::shared_ptr<Predictor>;

}  // namespace cg