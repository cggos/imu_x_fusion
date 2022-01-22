#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
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
    ekf_ptr_ = std::make_unique<EKF>(acc_n, gyr_n, acc_w, gyr_w);
    ekf_ptr_->state_ptr_->set_cov(sigma_pv, sigma_pv, sigma_rp, sigma_yaw, 0.02, 0.02);

    std::string topic_imu = "/imu/data";
    std::string topic_gps = "/fix";

    // imu_sub_ = nh.subscribe<sensor_msgs::Imu>(topic_imu, 10, boost::bind(&EKF::imu_callback, ekf_ptr_.get(), _1));
    imu_sub_ = nh.subscribe(topic_imu, 10, &FusionNode::imu_callback, this);
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
    ImuDataPtr imu_data_ptr = std::make_shared<ImuData>();
    imu_data_ptr->timestamp = imu_msg->header.stamp.toSec();
    imu_data_ptr->acc[0] = imu_msg->linear_acceleration.x;
    imu_data_ptr->acc[1] = imu_msg->linear_acceleration.y;
    imu_data_ptr->acc[2] = imu_msg->linear_acceleration.z;
    imu_data_ptr->gyr[0] = imu_msg->angular_velocity.x;
    imu_data_ptr->gyr[1] = imu_msg->angular_velocity.y;
    imu_data_ptr->gyr[2] = imu_msg->angular_velocity.z;

    if (!ekf_ptr_->imu_model_.push_data(imu_data_ptr, initialized_)) return;

    ekf_ptr_->predict(last_imu_ptr_, imu_data_ptr);

    last_imu_ptr_ = imu_data_ptr;
  }

  void gps_callback(const sensor_msgs::NavSatFixConstPtr &gps_msg);

 private:
  bool initialized_ = false;

  ImuDataConstPtr last_imu_ptr_;

  ros::Subscriber imu_sub_;
  ros::Subscriber gps_sub_;

  Eigen::Vector3d init_lla_;
  Eigen::Vector3d I_p_Gps_ = Eigen::Vector3d(0., 0., 0.);

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

  if (!initialized_) {
    if (!(initialized_ = ekf_ptr_->imu_model_.init(*ekf_ptr_->state_ptr_, gps_data_ptr->timestamp, last_imu_ptr_)))
      return;

    init_lla_ = gps_data_ptr->lla;

    printf("[cggos %s] System initialized.\n", __FUNCTION__);

    return;
  }

  // convert WGS84 to ENU frame
  Eigen::Vector3d p_G_Gps;
  cg::lla2enu(init_lla_, gps_data_ptr->lla, &p_G_Gps);

  // residual
  Eigen::Vector3d residual = p_G_Gps - (ekf_ptr_->state_ptr_->p_wb_ + ekf_ptr_->state_ptr_->Rwb_ * I_p_Gps_);

  std::cout << "---------------------" << std::endl;
  std::cout << "res: " << residual.transpose() << std::endl;

  // jacobian
  Eigen::Matrix<double, 3, kStateDim> H;
  H.setZero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  H.block<3, 3>(0, 6) = -ekf_ptr_->state_ptr_->Rwb_ * cg::skew_matrix(I_p_Gps_);

  // measurement covariance
  const Eigen::Matrix3d &R = gps_data_ptr->cov;

  Eigen::Matrix<double, kStateDim, 3> K;
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
    Eigen::Vector3d lla;
    cg::enu2lla(init_lla_, ekf_ptr_->state_ptr_->p_wb_, &lla);  // convert ENU state to lla
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
