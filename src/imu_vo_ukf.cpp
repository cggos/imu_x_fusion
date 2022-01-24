#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <boost/math/distributions/chi_squared.hpp>
#include <iostream>

#include "common/view.hpp"
#include "estimator/ukf.hpp"
#include "sensor/odom_6dof.hpp"

namespace cg {

ANGULAR_ERROR State::kAngError = ANGULAR_ERROR::LOCAL_ANGULAR_ERROR;

class UKFFusionNode {
 public:
  UKFFusionNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) : viewer_(nh) {
    std::string topic_vo = "/odom_vo";
    std::string topic_imu = "/imu0";

    nh.getParam("topic_vo", topic_vo);
    nh.getParam("topic_imu", topic_imu);

    std::cout << "topic_vo: " << topic_vo << std::endl;
    std::cout << "topic_imu: " << topic_imu << std::endl;

    double acc_n, gyr_n, acc_w, gyr_w, sigma_pv, sigma_rp, sigma_yaw;
    nh.param("acc_noise", acc_n, 1e-2);
    nh.param("gyr_noise", gyr_n, 1e-4);
    nh.param("acc_bias_noise", acc_w, 1e-6);
    nh.param("gyr_bias_noise", gyr_w, 1e-8);

    nh.param("init_sigma_pv", sigma_pv, 0.01);
    nh.param("init_sigma_rp", sigma_rp, 0.01);
    nh.param("init_sigma_yaw", sigma_yaw, 5.0);

    sigma_rp *= kDegreeToRadian;
    sigma_yaw *= kDegreeToRadian;

    ukf_ptr_ = std::make_unique<UKF>(acc_n, gyr_n, acc_w, gyr_w);
    ukf_ptr_->state_ptr_->set_cov(sigma_pv, sigma_pv, sigma_rp, sigma_yaw, acc_w, gyr_w);

    // imu_sub_ = nh.subscribe<sensor_msgs::Imu>(topic_imu, 10, boost::bind(&UKF::imu_callback, ukf_ptr_.get(), _1));
    imu_sub_ = nh.subscribe(topic_imu, 10, &UKFFusionNode::imu_callback, this);
    vo_sub_ = nh.subscribe(topic_vo, 10, &UKFFusionNode::vo_callback, this);

    Tcb = getTransformEigen(pnh, "cam0/T_cam_imu");

    for (int i = 1; i < 100; ++i) {
      boost::math::chi_squared chi_squared_dist(i);
      chi_squared_test_table_[i] = boost::math::quantile(chi_squared_dist, 0.05);
    }
  }

  ~UKFFusionNode() {}

  void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
    ImuDataPtr imu_data_ptr = std::make_shared<ImuData>();
    imu_data_ptr->timestamp = imu_msg->header.stamp.toSec();
    imu_data_ptr->acc[0] = imu_msg->linear_acceleration.x;
    imu_data_ptr->acc[1] = imu_msg->linear_acceleration.y;
    imu_data_ptr->acc[2] = imu_msg->linear_acceleration.z;
    imu_data_ptr->gyr[0] = imu_msg->angular_velocity.x;
    imu_data_ptr->gyr[1] = imu_msg->angular_velocity.y;
    imu_data_ptr->gyr[2] = imu_msg->angular_velocity.z;

    if (!ukf_ptr_->imu_model_.push_data(imu_data_ptr, initialized_)) return;

    ukf_ptr_->predict(last_imu_ptr_, imu_data_ptr);

    last_imu_ptr_ = imu_data_ptr;
  }

  void vo_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &vo_msg);

 private:
  bool initialized_ = false;

  ImuDataConstPtr last_imu_ptr_;

  ros::Subscriber imu_sub_;
  ros::Subscriber vo_sub_;

  Eigen::Isometry3d Tcb;
  Eigen::Isometry3d Tvw;
  Eigen::Isometry3d TvoB;  // for publish

  UKFPtr ukf_ptr_;
  Viewer viewer_;

  std::map<int, double> chi_squared_test_table_;
};

void UKFFusionNode::vo_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &vo_msg) {
  Eigen::Vector3d vo_p;
  Eigen::Quaterniond vo_q;
  vo_p.x() = vo_msg->pose.pose.position.x;
  vo_p.y() = vo_msg->pose.pose.position.y;
  vo_p.z() = vo_msg->pose.pose.position.z;
  vo_q.x() = vo_msg->pose.pose.orientation.x;
  vo_q.y() = vo_msg->pose.pose.orientation.y;
  vo_q.z() = vo_msg->pose.pose.orientation.z;
  vo_q.w() = vo_msg->pose.pose.orientation.w;

  Eigen::Isometry3d Tvo;  // VO in frame V --> Tc0cn
  Tvo.linear() = vo_q.toRotationMatrix();
  Tvo.translation() = vo_p;

  const Eigen::Matrix<double, kMeasDim, kMeasDim> &R =
      Eigen::Map<const Eigen::Matrix<double, kMeasDim, kMeasDim>>(vo_msg->pose.covariance.data());

  if (!initialized_) {
    if (!(initialized_ = ukf_ptr_->imu_model_.init(*ukf_ptr_->state_ptr_, vo_msg->header.stamp.toSec(), last_imu_ptr_)))
      return;

    ukf_ptr_->predicted_x_ = ukf_ptr_->state_ptr_->vec();
    ukf_ptr_->predicted_P_ = ukf_ptr_->state_ptr_->cov;

    Eigen::Isometry3d Tb0bm;
    Tb0bm.linear() = ukf_ptr_->state_ptr_->Rwb_;
    Tb0bm.translation().setZero();

    const Eigen::Isometry3d &Tc0cm = Tvo;

    Tvw = Tc0cm * Tcb * Tb0bm.inverse();  // c0 --> visual frame V, b0 --> world frame W

    printf("[cggos %s] System initialized.\n", __FUNCTION__);

    return;
  }

  std::cout << "---------------------" << std::endl;

  // predict measurement sigma points
  Eigen::Isometry3d Twb;
  Eigen::MatrixXd predicted_meas_sp_mat = Eigen::MatrixXd::Zero(kMeasDim, ukf_ptr_->sigma_points_num_);
  for (int i = 0; i < ukf_ptr_->sigma_points_num_; i++) {
    const auto &sp = ukf_ptr_->predicted_sp_mat_.col(i);

    Twb.translation() = sp.segment<3>(0);
    Twb.linear() = rot_vec_to_mat(sp.segment<3>(6));

    // measurement estimation h(x), Twb in frame V --> Tc0cn
    const Eigen::Isometry3d &Twb_in_V = Tvw * Twb * Tcb.inverse();

    predicted_meas_sp_mat.col(i).segment<3>(0) = Twb_in_V.translation();
    predicted_meas_sp_mat.col(i).segment<3>(3) = rot_mat_to_vec(Twb_in_V.linear());
  }

  // predict measurement mean
  Eigen::VectorXd predicted_z = Eigen::VectorXd::Zero(kMeasDim);
  for (int c = 0; c < ukf_ptr_->sigma_points_num_; c++) {
    predicted_z += ukf_ptr_->weights_mean_[c] * predicted_meas_sp_mat.col(c);
  }

  // predict measurement covariance
  Eigen::MatrixXd predicted_S = Eigen::MatrixXd::Zero(kMeasDim, kMeasDim);
  Eigen::VectorXd dz = Eigen::VectorXd(kMeasDim);
  for (int c = 0; c < ukf_ptr_->sigma_points_num_; c++) {
    dz = predicted_meas_sp_mat.col(c) - predicted_z;
    predicted_S += ukf_ptr_->weights_cov_[c] * dz * dz.transpose();
  }
  predicted_S += R;

  // compute Tc
  Eigen::VectorXd dx;
  Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(kStateDim, kMeasDim);
  for (int c = 0; c < ukf_ptr_->sigma_points_num_; c++) {
    dx = ukf_ptr_->predicted_sp_mat_.col(c) - ukf_ptr_->predicted_x_;
    dz = predicted_meas_sp_mat.col(c) - predicted_z;
    Tc += ukf_ptr_->weights_cov_[c] * dx * dz.transpose();
  }

  // update
  Eigen::Matrix<double, kMeasDim, 1> vec_vo;
  vec_vo.segment<3>(0) = Tvo.translation();
  vec_vo.segment<3>(3) = rot_mat_to_vec(Tvo.linear());
  dz = vec_vo - predicted_z;

  // chi-square gating test
  const float scale = 200;
  const int dof = kMeasDim;
  double chi2 = dz.transpose() * predicted_S.ldlt().solve(dz);
  std::cout << "chi2: " << chi2 << std::endl;
  if (chi2 >= scale * chi_squared_test_table_[dof]) {
    // TODO
    // return;
  }

  Eigen::MatrixXd K = Tc * predicted_S.inverse();

  ukf_ptr_->predicted_x_ = ukf_ptr_->predicted_x_ + K * dz;
  ukf_ptr_->predicted_P_ = ukf_ptr_->predicted_P_ - K * predicted_S * K.transpose();
  ukf_ptr_->predicted_P_ = 0.5 * (ukf_ptr_->predicted_P_ + ukf_ptr_->predicted_P_.transpose());

  // condition number
  // 解决：因观测误差较大，使P负定，致使后面P的Cholesky分解失败出现NaN，导致滤波器发散
  {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(ukf_ptr_->predicted_P_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd singularValues;
    singularValues.resize(svd.singularValues().rows(), 1);
    singularValues = svd.singularValues();
    double cond = singularValues(0, 0) / singularValues(singularValues.rows() - 1, 0);
    double max_cond_number = 1e5;
    std::cout << "cond num of P: " << std::abs(cond) << std::endl;
    if (std::abs(cond) > max_cond_number) {
      ukf_ptr_->predicted_P_ = ukf_ptr_->predicted_P_.diagonal().asDiagonal();
    }
  }

  ukf_ptr_->state_ptr_->from_vec(ukf_ptr_->predicted_x_);
  ukf_ptr_->state_ptr_->cov = ukf_ptr_->predicted_P_;

  std::cout << "acc bias: " << ukf_ptr_->state_ptr_->acc_bias.transpose() << std::endl;
  std::cout << "gyr bias: " << ukf_ptr_->state_ptr_->gyr_bias.transpose() << std::endl;
  std::cout << "---------------------" << std::endl;

  // view
  // for publish, Tvo in frame W --> Tb0bn
  TvoB = Tvw.inverse() * Tvo * Tcb;
  viewer_.publish_vo(*ukf_ptr_->state_ptr_, TvoB);
}

}  // namespace cg

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_vo_ukf_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  cg::UKFFusionNode fusion_node(nh, pnh);

  ros::spin();

  return 0;
}