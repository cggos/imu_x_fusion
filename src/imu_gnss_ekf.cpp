#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <deque>
#include <fstream>
#include <iostream>

#include "common/view.hpp"
#include "estimator/ekf.hpp"
#include "sensor/gnss.hpp"
#include "sensor/imu.hpp"

namespace cg {

ANGULAR_ERROR State::kAngError = ANGULAR_ERROR::LOCAL_ANGULAR_ERROR;

class FusionNode {
 public:
  FusionNode(ros::NodeHandle &nh) : viewer_(nh) {
    double acc_n, gyr_n, acc_w, gyr_w;
    nh.param("acc_noise", acc_n, 1e-2);
    nh.param("gyr_noise", gyr_n, 1e-4);
    nh.param("acc_bias_noise", acc_w, 1e-6);
    nh.param("gyr_bias_noise", gyr_w, 1e-8);

    const double sigma_pv = 10;
    const double sigma_rp = 10 * kDegreeToRadian;
    const double sigma_yaw = 100 * kDegreeToRadian;

    ekf_ptr_ = std::make_unique<EKF>();
    ekf_ptr_->state_ptr_->set_cov(sigma_pv, sigma_pv, sigma_rp, sigma_yaw, 0.02, 0.02);
    ekf_ptr_->predictor_ptr_ = std::make_shared<IMU>(ekf_ptr_->state_ptr_, acc_n, gyr_n, acc_w, gyr_w);
    ekf_ptr_->observer_ptr_ = std::make_shared<GNSS>();

    std::string topic_imu = "/imu/data";
    std::string topic_gps = "/fix";

    imu_sub_ = nh.subscribe<sensor_msgs::Imu>(topic_imu, 10, boost::bind(&FusionNode::imu_callback, this, _1));

    gps_sub_ = nh.subscribe(topic_gps, 10, &FusionNode::gps_callback, this);

    // log files
    file_gps_.open("fusion_gps.csv");
    file_state_.open("fusion_state.csv");
  }

  ~FusionNode() {
    if (file_gps_.is_open()) file_gps_.close();
    if (file_state_.is_open()) file_state_.close();
  }

  void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
    Eigen::Vector3d acc, gyr;
    acc[0] = imu_msg->linear_acceleration.x;
    acc[1] = imu_msg->linear_acceleration.y;
    acc[2] = imu_msg->linear_acceleration.z;
    gyr[0] = imu_msg->angular_velocity.x;
    gyr[1] = imu_msg->angular_velocity.y;
    gyr[2] = imu_msg->angular_velocity.z;

    ekf_ptr_->predict(std::make_shared<ImuData>(imu_msg->header.stamp.toSec(), acc, gyr));
  }

  void gps_callback(const sensor_msgs::NavSatFixConstPtr &gps_msg);

 private:
  ros::Subscriber imu_sub_;
  ros::Subscriber gps_sub_;

  EKFPtr ekf_ptr_;
  Viewer viewer_;

  std::ofstream file_gps_;
  std::ofstream file_state_;
};

void FusionNode::gps_callback(const sensor_msgs::NavSatFixConstPtr &gps_msg) {
  if (gps_msg->status.status != 2) {
    printf("[cggos %s] ERROR: Bad GPS Message!!!\n", __FUNCTION__);
    return;
  }

  GpsDataPtr gps_data_ptr = std::make_shared<GpsData>();
  gps_data_ptr->timestamp = gps_msg->header.stamp.toSec();
  gps_data_ptr->lla[0] = gps_msg->latitude;
  gps_data_ptr->lla[1] = gps_msg->longitude;
  gps_data_ptr->lla[2] = gps_msg->altitude;
  gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg->position_covariance.data());

  if (!ekf_ptr_->predictor_ptr_->inited_) {
    if (!ekf_ptr_->predictor_ptr_->init(gps_data_ptr->timestamp)) return;

    std::dynamic_pointer_cast<GNSS>(ekf_ptr_->observer_ptr_)->set_params(gps_data_ptr);

    printf("[cggos %s] System initialized.\n", __FUNCTION__);

    return;
  }

  std::cout << "---------------------" << std::endl;

  const Eigen::Isometry3d &Twb = ekf_ptr_->state_ptr_->pose();
  const auto &p_G_Gps = std::dynamic_pointer_cast<GNSS>(ekf_ptr_->observer_ptr_)->g2l(gps_data_ptr);

  const auto &residual = ekf_ptr_->observer_ptr_->measurement_residual(Twb.matrix(), p_G_Gps);

  std::cout << "res: " << residual.transpose() << std::endl;

  const auto &H = ekf_ptr_->observer_ptr_->measurement_jacobian(Twb.matrix(), p_G_Gps);

  Eigen::Matrix<double, kStateDim, 3> K;
  const Eigen::Matrix3d &R = gps_data_ptr->cov;
  ekf_ptr_->update_K(H, R, K);
  ekf_ptr_->update_P(H, R, K);
  *ekf_ptr_->state_ptr_ = *ekf_ptr_->state_ptr_ + K * residual;

  std::cout << "acc bias: " << ekf_ptr_->state_ptr_->acc_bias.transpose() << std::endl;
  std::cout << "gyr bias: " << ekf_ptr_->state_ptr_->gyr_bias.transpose() << std::endl;
  std::cout << "---------------------" << std::endl;

  // save data
  {
    viewer_.publish_gnss(*ekf_ptr_->state_ptr_);

    // save state p q lla
    const auto &lla = std::dynamic_pointer_cast<GNSS>(ekf_ptr_->observer_ptr_)->l2g(ekf_ptr_->state_ptr_->p_wb_);

    const Eigen::Quaterniond q_GI(ekf_ptr_->state_ptr_->Rwb_);
    file_state_ << std::fixed << std::setprecision(15) << ekf_ptr_->state_ptr_->timestamp << ", "
                << ekf_ptr_->state_ptr_->p_wb_[0] << ", " << ekf_ptr_->state_ptr_->p_wb_[1] << ", "
                << ekf_ptr_->state_ptr_->p_wb_[2] << ", " << q_GI.x() << ", " << q_GI.y() << ", " << q_GI.z() << ", "
                << q_GI.w() << ", " << lla[0] << ", " << lla[1] << ", " << lla[2] << std::endl;

    file_gps_ << std::fixed << std::setprecision(15) << gps_data_ptr->timestamp << ", " << gps_data_ptr->lla[0] << ", "
              << gps_data_ptr->lla[1] << ", " << gps_data_ptr->lla[2] << std::endl;
  }
}

}  // namespace cg

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_gnss_fusion");

  ros::NodeHandle nh;
  cg::FusionNode fusion_node(nh);

  ros::spin();

  return 0;
}
