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

    using Ptr = std::shared_ptr<Data>;
    using ConstPtr = std::shared_ptr<const Data>;
  };

  using Ptr = std::shared_ptr<Predictor>;

  Predictor() = default;

  Predictor(const Predictor &) = delete;

  Predictor(State::Ptr state_ptr) : state_ptr_(state_ptr) {}

  virtual ~Predictor() {}

  virtual bool init(double ts_meas) = 0;

  void process(Data::ConstPtr data_ptr, std::function<void(Data::ConstPtr, Data::ConstPtr)> func_predict = nullptr) {
    if (!data_ptr) return;

    if (!push_data(data_ptr)) return;

    if (func_predict != nullptr)
      func_predict(last_data_ptr_, data_ptr);
    else
      predict(last_data_ptr_, data_ptr);

    last_data_ptr_ = data_ptr;
  }

 protected:
  virtual bool push_data(Data::ConstPtr data_ptr) = 0;

  virtual void predict(Data::ConstPtr last_ptr, Data::ConstPtr curr_ptr) = 0;

 public:
  bool inited_ = false;

  State::Ptr state_ptr_;

 protected:
  Data::ConstPtr last_data_ptr_;
};

}  // namespace cg